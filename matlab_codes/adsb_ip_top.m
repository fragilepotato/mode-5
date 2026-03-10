function [rx_bits, valid_flag] = adsb_ip_top(rx_re, rx_im)
%#codegen
% ADSB_IP_TOP  IFF Transponder DSP — HDL Coder IP Core (integer version)
%
%  Pure int16/int32/uint8 arithmetic — no floating-point, no fi(), no MEX.
%  Direct HDL generation via codegen -config hdl.
%
%  Inputs:
%    rx_re — int16 column vector (6528 x 1), real part from AD9361 ADC
%    rx_im — int16 column vector (6528 x 1), imag part from AD9361 ADC
%
%  Outputs:
%    rx_bits    — uint8 row vector (1 x 88), decoded PPM bits (0 or 1)
%    valid_flag — logical scalar, true when valid ADS-B preamble detected
%
%  Integer range analysis (no overflow possible in int32):
%    mag_raw      : max = 2 * 32767 = 65534
%    running_sum  : max = 240 * 65534 = 15,728,160
%    mag (DC-rmvd): max = 65534
%    corr (96-tap): max = 96 * 65534 = 6,291,264
%    corr_scaled  : max = 6,291,264 / 256 = 24,575
%    sum_scaled   : max = 6433 * 24,575 = 158,079,775
%    SNR product  : max = 24,575 * 6433 = 158,079,775   (int32 OK)
%    SNR product  : max = 4 * 158,079,775 = 632,319,100  (int32 OK)
%
%  HDL mapping (ZedBoard xc7z020, Vivado 2023.2):
%    Stage 1: sliding-window accumulator + comparator
%    Stage 2: 96-tap FIR matched filter, peak tracker
%    Stage 3: per-symbol adder + comparator

% =====================================================================
%  COMPILE-TIME CONSTANTS
% =====================================================================
%  All loop bounds must be numeric literals for HDL Coder.
%  Named values here for documentation / computation reference.
%
%  FRAME_LEN   = 6528    (20+8+88+20)*12*4 samples
%  N_BITS      = 88      interrogation bits
%  SPS         = 12      samples per symbol (12 MSPS / 1 MHz)
%  DC_WIN      = 240     20 us * 12 samples
%  KERN_LEN    = 96      8 us * 12 samples
%  PULSE_W     = 6       0.5 us * 12 samples
%  HALF_SPS    = 6       SPS / 2
%  SEARCH_LEN  = 6433    FRAME_LEN - KERN_LEN + 1
%  DC_SHIFT    = 8       bitshift approx for /240 (actual /256, 6.7% err)
%  SNR_SCALE   = 8       bitshift to prevent overflow in SNR xmul
%  SNR_THRESH  = 4       minimum correlation SNR for valid preamble
%
%  Preamble pulse starts at 0, 1, 3.5, 4.5 us → samples 1, 13, 43, 55

% =====================================================================
%  STAGE 1 — Envelope extraction + sliding-window DC removal
%
%  L1 magnitude: |Re| + |Im| (avoids CORDIC sqrt)
%  DC baseline:  sliding-window sum >> 8 (≈ /256)
%  Clamp negative values to zero after DC subtraction
% =====================================================================

mag_raw = zeros(6528, 1, 'int32');
for i = 1:6528
    % int16 → int32 promotion, then L1 magnitude
    re_val = int32(rx_re(i));
    im_val = int32(rx_im(i));
    if re_val < int32(0)
        re_val = -re_val;
    end
    if im_val < int32(0)
        im_val = -im_val;
    end
    mag_raw(i) = re_val + im_val;
end

mag = zeros(6528, 1, 'int32');
running_sum = int32(0);
for i = 1:6528
    running_sum = running_sum + mag_raw(i);
    if i > 240
        running_sum = running_sum - mag_raw(i - 240);
    end
    % DC estimate: running_sum >> 8 ≈ running_sum / 256 ≈ /240
    dc  = bitshift(running_sum, -8);
    val = mag_raw(i) - dc;
    if val < int32(0)
        val = int32(0);
    end
    mag(i) = val;
end

% =====================================================================
%  STAGE 2 — Preamble matched filter + frame sync
%
%  Bipolar kernel: +1 at pulse positions, -1 elsewhere (96 taps total)
%  Preamble always produces POSITIVE correlation (kernel +1 aligns with
%  pulses), so we track max(c), NOT max(abs(c)).
%  SNR check uses bitshift-scaled cross-multiply to stay in int32.
% =====================================================================

% Build bipolar kernel (+1 at preamble pulse positions, -1 elsewhere)
kernel = zeros(96, 1, 'int32');
for i = 1:96
    kernel(i) = int32(-1);
end
%  Pulse at 0 us → samples 1..6
for j = 0:5, kernel(1  + j) = int32(1); end
%  Pulse at 1 us → samples 13..18
for j = 0:5, kernel(13 + j) = int32(1); end
%  Pulse at 3.5 us → samples 43..48
for j = 0:5, kernel(43 + j) = int32(1); end
%  Pulse at 4.5 us → samples 55..60
for j = 0:5, kernel(55 + j) = int32(1); end

% Slide kernel over envelope
best_corr       = int32(-2147483647);   % start at min so any real peak wins
best_pos        = int32(1);
corr_sum_scaled = int32(0);             % accumulates |corr| >> 8

for i = 1:6433   % SEARCH_LEN = 6528 - 96 + 1
    c = int32(0);
    for j = 1:96
        c = c + mag(i + j - 1) * kernel(j);
    end

    % Accumulate scaled |corr| for SNR denominator
    c_for_abs = c;
    if c_for_abs < int32(0)
        c_for_abs = -c_for_abs;
    end
    corr_sum_scaled = corr_sum_scaled + bitshift(c_for_abs, -8);

    % Track maximum positive correlation
    if c > best_corr
        best_corr = c;
        best_pos  = int32(i);
    end
end

% SNR check (division-free cross-multiply):
%   snr = best_corr / mean(|corr|) >= SNR_THRESH
%   best_corr / (corr_sum_scaled * 256 / 6433) >= 4
%   (best_corr >> 8) * 6433 >= 4 * corr_sum_scaled
best_corr_scaled = bitshift(best_corr, -8);
snr_left  = best_corr_scaled * int32(6433);
snr_right = int32(4) * corr_sum_scaled;
valid_flag = snr_left >= snr_right;

% Data payload starts after the matched preamble
data_start = best_pos + int32(96);
max_safe   = int32(6528) - int32(88) * int32(12);  % 5472
if data_start < int32(1) || data_start > max_safe
    data_start = int32(1);
    valid_flag = false;
end

% =====================================================================
%  STAGE 3 — PPM bit decisions
%
%  Per symbol: compare energy in first half vs second half.
%  sum(mag[i0 .. i0+5]) vs sum(mag[i0+6 .. i0+11])
%  If first_half > second_half → bit = 1,  else bit = 0
% =====================================================================

rx_bits = zeros(1, 88, 'uint8');

if valid_flag
    for k = int32(0):int32(87)
        i0 = data_start + k * int32(12);
        if i0 + int32(11) <= int32(6528)
            sum_first  = int32(0);
            sum_second = int32(0);
            for s = int32(0):int32(5)
                sum_first  = sum_first  + mag(i0 + s);
                sum_second = sum_second + mag(i0 + int32(6) + s);
            end
            if sum_first > sum_second
                rx_bits(k + int32(1)) = uint8(1);
            end
        end
    end
end

end
