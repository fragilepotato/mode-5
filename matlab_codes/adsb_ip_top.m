function [rx_bits, valid_flag] = adsb_ip_top(rx_iq)
%#codegen
% ADSB_IP_TOP  IFF Transponder DSP — HDL Coder IP Core entry point
%
%  Top-level wrapper for the three DSP stages that decode an IFF/ADS-B
%  interrogation frame from raw IQ samples.  Every operation is
%  HDL-synthesizable: no movmean, xcorr, median, eps, or dynamic alloc.
%
%  Input:
%    rx_iq  — complex double column vector, 6528 x 1
%             (IQ frame captured at 12 MSPS from AD9361)
%
%  Outputs:
%    rx_bits    — 1 x 88 double, decoded interrogation bits
%    valid_flag — logical scalar, true when a valid preamble is detected
%
%  HDL mapping (ZedBoard xc7z020, Vivado 2023.2):
%    Stage 1 (preprocess)  — sliding-window accumulator + comparator
%    Stage 2 (framesync)   — 96-tap FIR matched filter, peak tracker
%    Stage 3 (decode_bits) — per-symbol adder + comparator, no divider
%
%  Replaces the prototype functions:
%    adsb_preprocess  → Stage 1 (movmean   replaced by manual sliding window)
%    adsb_framesync   → Stage 2 (xcorr     replaced by direct correlation loop)
%                                (median    replaced by sum-based SNR check)
%    adsb_decode_bits → Stage 3 (unchanged logic, explicit inner loops)

% =====================================================================
%  COMPILE-TIME CONSTANTS
% =====================================================================
FRAME_LEN  = 6528;          % (20+8+88+20)*12*4  samples per capture
N_BITS     = 88;            % interrogation frame length in bits
SPS        = 12;            % samples per symbol  (12 MSPS / 1 MHz)
DC_WIN     = 240;           % 20 us * 12 = sliding-window length
KERN_LEN   = 96;            % 8 us * 12  = preamble kernel length
PULSE_W    = 6;             % 0.5 us * 12 = single pulse width
HALF_SPS   = 6;             % SPS / 2  (half-symbol for PPM decision)
SEARCH_LEN = FRAME_LEN - KERN_LEN + 1;   % 6433 correlation positions
SNR_THRESH = 4.0;           % minimum SNR to declare valid preamble
TINY       = 1e-10;         % replaces eps (not HDL-compatible)

% Preamble pulse start indices (1-based):
%   0 us -> sample 1,  1 us -> 13,  3.5 us -> 43,  4.5 us -> 55
PULSE_STARTS = [1, 13, 43, 55];

% =====================================================================
%  STAGE 1 — Envelope extraction + DC removal
%
%  HDL: |Re|+|Im| (two abs + adder, no CORDIC sqrt)
%       Sliding-window accumulator (shift register + adder)
%       Constant-divisor reciprocal multiply (1/DC_WIN)
%       Comparator for clamp-to-zero
% =====================================================================

% L1 magnitude  (avoids expensive CORDIC for sqrt(re^2+im^2))
mag_raw = zeros(FRAME_LEN, 1);
for i = 1:FRAME_LEN
    mag_raw(i) = abs(real(rx_iq(i))) + abs(imag(rx_iq(i)));
end

% Sliding-window DC baseline subtraction (replaces movmean)
%   Always divides by DC_WIN (constant) — maps to reciprocal multiplier.
%   First DC_WIN samples get a slightly high DC estimate; acceptable for
%   frame detection since the preamble is never at sample 1.
mag = zeros(FRAME_LEN, 1);
running_sum = 0.0;
for i = 1:FRAME_LEN
    running_sum = running_sum + mag_raw(i);
    if i > DC_WIN
        running_sum = running_sum - mag_raw(i - DC_WIN);
    end
    dc  = running_sum / DC_WIN;
    val = mag_raw(i) - dc;
    if val < 0
        val = 0;
    end
    mag(i) = val;
end

% =====================================================================
%  STAGE 2 — Preamble matched filter + frame sync
%
%  HDL: Bipolar kernel stored in ROM (96 coefficients, +1/-1)
%       Direct dot-product correlation (96-tap FIR per position)
%       Running peak tracker (comparator + register)
%       Division-free SNR check: peak*N >= thresh*sum
% =====================================================================

% Build bipolar preamble kernel (+1 at pulse positions, -1 elsewhere)
kernel = zeros(KERN_LEN, 1);
for i = 1:KERN_LEN
    kernel(i) = -1;
end
for p = 1:4
    for j = 0:PULSE_W-1
        idx = PULSE_STARTS(p) + j;
        if idx >= 1 && idx <= KERN_LEN
            kernel(idx) = 1;
        end
    end
end

% Slide kernel over envelope, track MAXIMUM correlation (not abs)
%   The preamble always produces a positive correlation because the kernel
%   +1 positions align with actual pulses.  Using abs() would let a large
%   NEGATIVE correlation (signal energy on -1 kernel positions) in the data
%   region steal the peak — causing wrong data_start and ~50% BER.
best_corr      = -1e30;   % must start negative so any real peak wins
best_pos       = 1;
corr_abs_total = 0.0;

for i = 1:SEARCH_LEN
    c = 0.0;
    for j = 1:KERN_LEN
        c = c + mag(i + j - 1) * kernel(j);
    end
    corr_abs_total = corr_abs_total + abs(c);   % SNR denominator
    if c > best_corr                            % max(c), NOT max(abs(c))
        best_corr = c;
        best_pos  = i;
    end
end

% SNR check WITHOUT division (cross-multiply the inequality):
%   snr = best_corr / (corr_abs_total / SEARCH_LEN) >= SNR_THRESH
%   =>   best_corr * SEARCH_LEN >= SNR_THRESH * corr_abs_total
valid_flag = (best_corr * SEARCH_LEN) >= (SNR_THRESH * corr_abs_total);

% Data payload starts right after the matched preamble
data_start = best_pos + KERN_LEN;
max_safe   = FRAME_LEN - N_BITS * SPS;
if data_start < 1 || data_start > max_safe
    data_start = 1;
    valid_flag = false;
end

% =====================================================================
%  STAGE 3 — PPM bit decisions
%
%  HDL: Per-symbol accumulate-and-compare
%       Two 6-tap adders (first half vs second half)
%       Single comparator — no divider needed
% =====================================================================
rx_bits = zeros(1, N_BITS);

if valid_flag
    for k = 0:N_BITS-1
        i0 = data_start + k * SPS;
        if i0 + SPS - 1 <= FRAME_LEN
            sum_first  = 0.0;
            sum_second = 0.0;
            for s = 0:HALF_SPS-1
                sum_first  = sum_first  + mag(i0 + s);
                sum_second = sum_second + mag(i0 + HALF_SPS + s);
            end
            if sum_first > sum_second
                rx_bits(k + 1) = 1;
            end
        end
    end
end

end
