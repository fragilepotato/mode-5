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
KERN_LEN    = int32(96);      % 8 us preamble
SPS         = int32(12);      % samples per symbol
HALF_SPS    = int32(6);       % half symbol
N_BITS      = int32(88);      % interrogation frame length
SNR_THRESH  = int32(4);       % minimum SNR

% Preamble kernel: +1 at pulse positions, -1 elsewhere
% Pulses at 0, 1, 3.5, 4.5 us → samples 1,13,43,55 (each 6 wide)
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
persistent mag_sr;      % int32, circular buffer for DC window
persistent dc_acc;      % int32, running sum for DC baseline
persistent wr_idx;      % int32, write index into mag_sr (1-based)
persistent fill_count;  % int32, how many samples written so far

% Stage 2: Matched filter shift register
persistent corr_sr;     % int32, 96-deep shift register for envelope
persistent corr_wr;     % int32, write index into corr_sr
persistent corr_fill;   % int32, how many envelope samples so far
persistent best_corr;   % int32, best correlation seen in current search
persistent corr_sum;    % int32, accumulated |corr| >> 8 for SNR calc
persistent corr_count;  % int32, number of correlation values computed
persistent preamble_locked; % logical, true when searching complete
persistent snr_pass;    % logical, SNR exceeded threshold

% Stage 3: Bit extraction
persistent bit_phase;   % int32, sample counter within current symbol
persistent bit_index;   % int32, which bit we're extracting (0-based)
persistent sum_first;   % int32, energy in first half of symbol
persistent sum_second;  % int32, energy in second half of symbol
persistent decoding;    % logical, true while extracting 88 bits

% Frame management
persistent armed;       % logical, waiting for a new frame

if isempty(mag_sr)
    mag_sr      = zeros(240, 1, 'int32');
    dc_acc      = int32(0);
    wr_idx      = int32(1);
    fill_count  = int32(0);

    corr_sr     = zeros(96, 1, 'int32');
    corr_wr     = int32(1);
    corr_fill   = int32(0);
    best_corr   = int32(-2147483647);
    corr_sum    = int32(0);
    corr_count  = int32(0);
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

% Sliding-window DC accumulator
old_val = mag_sr(wr_idx);
mag_sr(wr_idx) = mag_raw;
dc_acc = dc_acc + mag_raw - old_val;

% Advance circular index
wr_idx = wr_idx + int32(1);
if wr_idx > DC_WIN
    wr_idx = int32(1);
end

fill_count = fill_count + int32(1);

% DC baseline via bitshift: dc_acc >> 8 ≈ dc_acc / 256 ≈ /240
dc_est = bitshift(dc_acc, -8);
mag_val = mag_raw - dc_est;
if mag_val < int32(0)
    mag_val = int32(0);
end

% =====================================================================
%  STAGE 2 — Preamble matched filter (1 sample per clock)
%
%  Maintains a 96-deep shift register of envelope samples.
%  Once filled, computes 96-tap dot product every clock cycle.
%  After a configurable search window (preamble_locked=false → searching),
%  locks onto the best peak and starts bit extraction.
% =====================================================================

if ~preamble_locked && armed
    % Write envelope into correlation shift register
    corr_sr(corr_wr) = mag_val;
    corr_wr = corr_wr + int32(1);
    if corr_wr > KERN_LEN
        corr_wr = int32(1);
    end
    corr_fill = corr_fill + int32(1);

    % Once we have 96 samples, compute correlation every clock
    if corr_fill >= KERN_LEN
        c = int32(0);
        rd = corr_wr;  % oldest sample is at current write position
        for j = 1:96
            if rd > 96
                rd = int32(1);
            end
            c = c + corr_sr(rd) * KERNEL(j);
            rd = rd + int32(1);
        end

        % Accumulate for SNR
        c_abs = c;
        if c_abs < int32(0), c_abs = -c_abs; end
        corr_sum = corr_sum + bitshift(c_abs, -8);
        corr_count = corr_count + int32(1);

        % Track best positive correlation
        if c > best_corr
            best_corr = c;
        end

        % After processing enough samples (~6400), lock and check SNR
        % We search for at most 6433 positions (one full frame)
        if corr_count >= int32(6433)
            preamble_locked = true;

            % SNR check: best_corr >> 8 * 6433 >= 4 * corr_sum
            best_scaled = bitshift(best_corr, -8);
            snr_left  = best_scaled * int32(6433);
            snr_right = SNR_THRESH * corr_sum;
            snr_pass  = snr_left >= snr_right;

            % Start bit extraction
            if snr_pass
                decoding   = true;
                bit_index  = int32(0);
                bit_phase  = int32(0);
                sum_first  = int32(0);
                sum_second = int32(0);
            end
        end
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

            % Reset state for next frame
            armed          = true;
            preamble_locked = false;
            best_corr      = int32(-2147483647);
            corr_sum       = int32(0);
            corr_count     = int32(0);
            corr_fill      = int32(0);
            corr_wr        = int32(1);
            fill_count     = int32(0);
            snr_pass       = false;
        end

        % Reset symbol accumulators
        bit_phase  = int32(0);
        sum_first  = int32(0);
        sum_second = int32(0);
    end
end

end
