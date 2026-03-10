function [bit_out, bit_valid, frame_done, frame_valid] = adsb_ip_top(re_in, im_in, sample_valid)
%#codegen
% ADSB_IP_TOP  Streaming IFF/ADS-B decoder — 1 sample per clock cycle
%
%  Sample-at-a-time architecture for HDL Coder synthesis.
%  Uses persistent state to implement sliding-window DC removal,
%  preamble matched filter, and PPM bit extraction across clock cycles.
%
%  Inputs  (per clock cycle):
%    re_in        — int16 scalar, real part of IQ sample from AD9361 ADC
%    im_in        — int16 scalar, imag part of IQ sample from AD9361 ADC
%    sample_valid — logical scalar, high when re_in/im_in are valid data
%
%  Outputs (per clock cycle):
%    bit_out      — uint8 scalar, decoded bit value (0 or 1)
%    bit_valid    — logical scalar, high for 1 clock when bit_out is valid
%    frame_done   — logical scalar, high for 1 clock after 88th bit decoded
%    frame_valid  — logical scalar, high with frame_done if preamble SNR passed
%
%  IO pin count: 16 + 16 + 1 + 8 + 1 + 1 + 1 = 44 pins (well under 5000)
%
%  Architecture:
%    Stage 1 (every sample): L1 envelope, sliding-window DC removal
%    Stage 2 (every sample): 96-tap matched-filter correlation, peak tracker
%    Stage 3 (after preamble lock): PPM bit decisions every 12 samples
%
%  Target: ZedBoard xc7z020clg484-1, Vivado 2023.2

% =====================================================================
%  CONSTANTS
% =====================================================================
DC_WIN      = int32(240);     % 20 us at 12 MSPS
KERN_LEN    = int32(96);      % 8 us preamble = 96 samples at 12 MSPS
SPS         = int32(12);      % samples per symbol (1 us at 12 MSPS)
HALF_SPS    = int32(6);       % half symbol
N_BITS      = int32(88);      % interrogation frame length
% SNR threshold: lock when corr > LOCK_SCALE/256 * dc_level * KERN_LEN
% Tune: 32/256 = 12.5% above DC baseline (robust for SNR > ~10 dB)
LOCK_SCALE  = int32(32);

% Preamble kernel: +1 at pulse positions, -1 elsewhere
% Pulses at 0-0.5us, 1-1.5us, 3.5-4us, 4.5-5us → each is 6 samples wide
KERNEL = int32([ ...
    1, 1, 1, 1, 1, 1,-1,-1,-1,-1,-1,-1, ...   % 0-1 us
    1, 1, 1, 1, 1, 1,-1,-1,-1,-1,-1,-1, ...   % 1-2 us
   -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1, ...   % 2-3 us
   -1,-1,-1,-1,-1,-1, 1, 1, 1, 1, 1, 1, ...   % 3-4 us
    1, 1, 1, 1, 1, 1,-1,-1,-1,-1,-1,-1, ...   % 4-5 us
   -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1, ...   % 5-6 us
   -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1, ...   % 6-7 us
   -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1]);     % 7-8 us

% =====================================================================
%  PERSISTENT STATE
% =====================================================================

% Stage 1: DC removal shift register + accumulator
persistent mag_sr;      % int32[240], circular buffer for DC window
persistent dc_acc;      % int32, running sum for DC baseline
persistent wr_idx;      % int32, write index into mag_sr (1-based)

% Stage 2: Matched filter shift register + immediate-lock peak detector
persistent corr_sr;     % int32[96], shift register of DC-removed envelope
persistent corr_wr;     % int32, write index into corr_sr (1-based)
persistent corr_fill;   % int32, count of samples in corr_sr so far
persistent prev_corr;   % int32, correlation value from previous clock
persistent preamble_locked; % logical
persistent snr_pass;    % logical

% Stage 3: Bit extraction
persistent bit_phase;   % int32, sample counter within current symbol (0..SPS-1)
persistent bit_index;   % int32, how many bits decoded so far (0-based)
persistent sum_first;   % int32, energy in first half of symbol
persistent sum_second;  % int32, energy in second half of symbol
persistent decoding;    % logical

% Frame management
persistent armed;       % logical

if isempty(mag_sr)
    mag_sr      = zeros(240, 1, 'int32');
    dc_acc      = int32(0);
    wr_idx      = int32(1);

    corr_sr     = zeros(96, 1, 'int32');
    corr_wr     = int32(1);
    corr_fill   = int32(0);
    prev_corr   = int32(0);
    preamble_locked = false;
    snr_pass    = false;

    bit_phase   = int32(0);
    bit_index   = int32(0);
    sum_first   = int32(0);
    sum_second  = int32(0);
    decoding    = false;

    armed       = true;
end

% =====================================================================
%  DEFAULT OUTPUTS
% =====================================================================
bit_out     = uint8(0);
bit_valid   = false;
frame_done  = false;
frame_valid = false;

if ~sample_valid
    return;
end

% =====================================================================
%  STAGE 1 — Envelope + DC removal (1 sample per clock)
% =====================================================================

% L1 magnitude: |Re| + |Im|
re_abs = int32(re_in);
im_abs = int32(im_in);
if re_abs < int32(0), re_abs = -re_abs; end
if im_abs < int32(0), im_abs = -im_abs; end
mag_raw = re_abs + im_abs;

% Sliding-window DC accumulator (circular buffer, width=240)
old_val = mag_sr(wr_idx);
mag_sr(wr_idx) = mag_raw;
dc_acc = dc_acc + mag_raw - old_val;
wr_idx = wr_idx + int32(1);
if wr_idx > DC_WIN
    wr_idx = int32(1);
end

% DC-subtracted envelope (floor at 0)
% dc_acc >> 8 ≈ dc_acc/256 ≈ mean over 240 samples
dc_est = bitshift(dc_acc, -8);
mag_val = mag_raw - dc_est;
if mag_val < int32(0)
    mag_val = int32(0);
end

% =====================================================================
%  STAGE 2 — Preamble matched filter with immediate peak-lock
%
%  On every sample, slide a new envelope value into the 96-tap shift
%  register and compute the dot product with KERNEL (±1 only → add/sub).
%  Lock immediately when:
%    (a) correlation was rising last clock (prev_corr was positive), AND
%    (b) current correlation is lower than prev (peak just passed), AND
%    (c) prev_corr exceeds LOCK_SCALE/256 * dc_acc (= ~12.5% above DC)
%
%  When locked, decoding starts on the NEXT sample (the preamble tail
%  is already 96 samples behind us; the data payload starts now).
% =====================================================================

if ~preamble_locked && armed
    % Push new envelope sample into circular shift register
    corr_sr(corr_wr) = mag_val;
    corr_wr = corr_wr + int32(1);
    if corr_wr > KERN_LEN
        corr_wr = int32(1);
    end
    corr_fill = corr_fill + int32(1);

    if corr_fill >= KERN_LEN
        % Compute 96-tap correlation (KERNEL is ±1 → no multiplier needed)
        c = int32(0);
        rd = corr_wr;   % oldest sample = current write position (just wrapped)
        for j = 1:96
            if rd > 96
                rd = int32(1);
            end
            if KERNEL(j) > int32(0)
                c = c + corr_sr(rd);
            else
                c = c - corr_sr(rd);
            end
            rd = rd + int32(1);
        end

        % Lock condition: peak just passed (prev > curr) AND prev > threshold
        % Only evaluate after DC window has fully warmed up (corr_fill > DC_WIN).
        % Threshold = LOCK_SCALE * dc_est prevents false locks on noise.
        % With dc_est = mean envelope, threshold captures ~12.5% above noise floor.
        threshold = LOCK_SCALE * bitshift(dc_acc, -8);

        if (corr_fill > DC_WIN) && ...
           (prev_corr > c) && (prev_corr > int32(0)) && (prev_corr > threshold)
            preamble_locked = true;
            snr_pass        = true;   % threshold crossing IS the SNR check
            decoding        = true;
            bit_index       = int32(0);
            bit_phase       = int32(0);
            sum_first       = int32(0);
            sum_second      = int32(0);
        end

        prev_corr = c;
    end

% =====================================================================
%  STAGE 3 — PPM bit decisions (during decoding phase)
%
%  After preamble lock, the next N_BITS * SPS = 1056 samples are the
%  data payload.  For each 12-sample symbol, accumulate energy in first
%  half (samples 0-5) and second half (samples 6-11), then compare.
% =====================================================================

elseif decoding

    if bit_phase < HALF_SPS
        sum_first = sum_first + mag_val;
    else
        sum_second = sum_second + mag_val;
    end

    bit_phase = bit_phase + int32(1);

    if bit_phase >= SPS
        % Symbol complete — decide bit
        if sum_first > sum_second
            bit_out = uint8(1);
        else
            bit_out = uint8(0);
        end
        bit_valid = true;

        bit_index = bit_index + int32(1);

        if bit_index >= N_BITS
            % All 88 bits decoded — frame complete
            frame_done  = true;
            frame_valid = snr_pass;
            decoding    = false;

            % Reset for next frame
            armed           = true;
            preamble_locked = false;
            prev_corr       = int32(0);
            corr_fill       = int32(0);
            corr_wr         = int32(1);
            snr_pass        = false;
        end

        % Reset symbol accumulators
        bit_phase  = int32(0);
        sum_first  = int32(0);
        sum_second = int32(0);
    end
end

end
