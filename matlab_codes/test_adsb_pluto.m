%% ADS-B TX/ACK — ADALM Pluto SDR (Interrogator side)
%
% Each cycle:
%   1. TX ADS-B waveform on 1090 MHz for TX_DURATION seconds
%      (ZedBoard receives and decodes during this window)
%   2. Stop TX, listen on 1030 MHz for ZedBoard's ACK
%      (ZedBoard echoes the decoded message back on 1030 MHz)
%   3. Decode ACK and report match — this is how you confirm the
%      ZedBoard received correctly without needing its console.
%
% Run this FIRST, then run test_adsb_zedboard.m.
clear all; clc;

% =====================================================================
%  CONFIGURATION
% =====================================================================
ip_pluto      = 'ip:192.168.2.1';
fc_tx         = 1090e6;   % Forward link: Pluto TX -> ZedBoard RX
fc_rx         = 1030e6;   % Return  link: ZedBoard TX -> Pluto RX
fs            = 12e6;     % 12 MSPS
tx_gain       = -20;      % dB
rx_gain       = 10;       % dB

TX_DURATION   = 3.0;      % seconds to transmit (gives ZedBoard time to decode)
RX_DURATION   = 5.0;      % seconds to listen for ACK

MAX_CYCLES     = 5;
CYCLE_INTERVAL = 5;       % seconds between cycles

msg_hex = '8D4840D6202CC371C32CE0576098';

% =====================================================================
%  BUILD TX WAVEFORM
% =====================================================================
[tx_wave, tx_bits] = adsb_modulate(msg_hex, fs);

% ACK capture buffer: 4× one ADS-B frame length
ack_frame_samples = round((20 + 8 + 112 + 20) * fs * 1e-6) * 4;

fprintf('========================================================\n');
fprintf('   ADS-B TX/ACK — ADALM Pluto\n');
fprintf('========================================================\n');
fprintf('Pluto       : %s\n', ip_pluto);
fprintf('TX Freq     : %.0f MHz (ADS-B forward)\n', fc_tx/1e6);
fprintf('RX Freq     : %.0f MHz (ZedBoard ACK)\n', fc_rx/1e6);
fprintf('Fs          : %.0f MSPS\n', fs/1e6);
fprintf('Message     : %s  (%d bits)\n', msg_hex, length(tx_bits));
fprintf('TX window   : %.1f s   RX window: %.1f s\n', TX_DURATION, RX_DURATION);
fprintf('Cycles      : %d  |  Interval: %d s\n', MAX_CYCLES, CYCLE_INTERVAL);
fprintf('========================================================\n\n');

% =====================================================================
%  MAIN CYCLE LOOP
% =====================================================================
results = zeros(MAX_CYCLES, 3);  % [ack_found, bit_errors, ack_snr]

for cycle = 1:MAX_CYCLES
    fprintf('=== Cycle %d/%d  [%s] ===\n', cycle, MAX_CYCLES, datestr(now,'HH:MM:SS'));

    % ------------------------------------------------------------------
    %  STEP 1 — Transmit ADS-B on 1090 MHz
    % ------------------------------------------------------------------
    fprintf('  [TX] Sending ADS-B on %.0f MHz for %.1f s ...\n', fc_tx/1e6, TX_DURATION);
    try
        ptx = sdrtx('Pluto', 'RadioID', ip_pluto, ...
            'CenterFrequency', fc_tx, 'BasebandSampleRate', fs);
        ptx.Gain = tx_gain;
        transmitRepeat(ptx, tx_wave);
        pause(TX_DURATION);   % hold TX while ZedBoard decodes
        release(ptx);
        fprintf('  [TX] Done.\n');
    catch ME
        fprintf('  [TX ERROR] %s\n', ME.message);
        try, release(ptx); catch, end
        results(cycle,:) = [0, length(tx_bits), 0];
        if cycle < MAX_CYCLES, pause(CYCLE_INTERVAL); end
        continue;
    end

    % ------------------------------------------------------------------
    %  STEP 2 — Listen for ZedBoard ACK on 1030 MHz
    % ------------------------------------------------------------------
    fprintf('  [RX] Listening for ACK on %.0f MHz (%.1f s window) ...\n', ...
        fc_rx/1e6, RX_DURATION);
    ack_found  = false;
    bit_errors = length(tx_bits);
    snr_ack    = 0;

    try
        prx = sdrrx('Pluto', 'RadioID', ip_pluto, ...
            'CenterFrequency', fc_rx, 'BasebandSampleRate', fs, ...
            'SamplesPerFrame', ack_frame_samples, ...
            'OutputDataType', 'double');
        prx.GainSource = 'Manual';
        prx.Gain = rx_gain;

        pause(1.0);                      % settle
        t_deadline = tic;
        while toc(t_deadline) < (RX_DURATION - 1.0)
            ack_raw = prx();

            mx = max(abs(ack_raw));
            if mx < 0.02
                continue;               % nothing there yet
            end

            mag = adsb_preprocess(ack_raw, fs);
            [data_start, corr_peak, snr_ratio] = adsb_framesync(mag, fs);

            if snr_ratio >= 4
                ack_bits  = adsb_decode_bits(mag, data_start, fs, 112);
                ack_hex   = bits2hex(ack_bits);
                snr_ack   = snr_ratio;
                bit_errors = sum(ack_bits(1:length(tx_bits)) ~= tx_bits);
                ack_found  = true;

                fprintf('  [ACK] Received from ZedBoard:\n');
                fprintf('        Sent    : %s\n', msg_hex);
                fprintf('        Echo    : %s\n', ack_hex);
                if bit_errors == 0
                    fprintf('        MATCH — ZedBoard decode was PERFECT.\n');
                else
                    fprintf('        %d/%d bit errors in ACK echo.\n', ...
                        bit_errors, length(tx_bits));
                end
                break;
            end
        end

        release(prx);

        if ~ack_found
            fprintf('  [ACK] No ACK received — ZedBoard may not have decoded frame.\n');
        end
    catch ME
        fprintf('  [RX ERROR] %s\n', ME.message);
        try, release(prx); catch, end
    end

    results(cycle,:) = [ack_found, bit_errors, snr_ack];

    if cycle < MAX_CYCLES
        fprintf('  Next cycle in %d s ...\n\n', CYCLE_INTERVAL);
        pause(CYCLE_INTERVAL);
    end
end

% =====================================================================
%  SUMMARY
% =====================================================================
n_ack   = sum(results(:,1));
n_match = sum(results(:,1) & results(:,2)==0);
fprintf('\n========================================================\n');
fprintf('                    SUMMARY\n');
fprintf('========================================================\n');
fprintf('ACK received   : %d/%d cycles\n', n_ack, MAX_CYCLES);
fprintf('Perfect match  : %d/%d cycles\n', n_match, MAX_CYCLES);
if n_match == MAX_CYCLES
    fprintf('ALL CYCLES PERFECT — RF link confirmed bidirectional.\n');
elseif n_ack > 0
    fprintf('Partial success — check ZedBoard RX gain.\n');
else
    fprintf('No ACK — check ZedBoard is running and cable connected.\n');
end
fprintf('========================================================\n');

% =====================================================================
%  ADS-B MODULATOR  (Hex string -> complex PPM I/Q waveform)
% =====================================================================
function [waveform, bits] = adsb_modulate(hexStr, fs)
    % Convert hex string to bits
    bits = [];
    for i = 1:length(hexStr)
        val  = hex2dec(hexStr(i));
        bits = [bits, bitget(val, 4:-1:1)];
    end

    sps = round(fs * 1e-6);       % samples per microsecond (12 at 12 MSPS)

    % Preamble (8 µs): pulses at 0, 1, 3.5, 4.5 µs (each 0.5 µs wide)
    preamble_len = round(8 * sps);
    preamble     = zeros(1, preamble_len);
    pulse_len    = round(0.5 * sps);
    for t_us = [0, 1, 3.5, 4.5]
        t0 = round(t_us * sps) + 1;
        t1 = min(t0 + pulse_len - 1, preamble_len);
        preamble(t0:t1) = 1;
    end

    % Data: PPM encoding (1 µs per bit)
    %   bit=1 -> pulse in first  half-bit (0-0.5 µs up, 0.5-1 µs down)
    %   bit=0 -> pulse in second half-bit (0-0.5 µs down, 0.5-1 µs up)
    data = [];
    for b = bits
        pat = zeros(1, sps);
        mid = floor(sps / 2);
        if b == 1
            pat(1:mid) = 1;
        else
            pat(mid+1:end) = 1;
        end
        data = [data, pat];
    end

    % Guard silence before/after frame (20 µs each side)
    guard = zeros(1, sps * 20);
    sig   = double([guard, preamble, data, guard]);

    % Normalize and force complex (PlutoSDR rejects real waveforms)
    sig      = sig / max(abs(sig)) * 0.9;
    waveform = sig.' + 1i * 1e-12 * ones(length(sig), 1);
end

% =====================================================================
%  SHARED DSP FUNCTIONS  (same as test_adsb_zedboard.m — IP core targets)
% =====================================================================

function mag = adsb_preprocess(rx_iq, fs)
    sps    = round(fs / 1e6);
    mag    = abs(rx_iq(:));
    dc_win = round(20 * sps);
    mag    = mag - movmean(mag, dc_win);
    mag    = max(mag, 0);
    mag    = mag / (max(mag) + eps);
end

function [data_start, corr_peak, snr_ratio] = adsb_framesync(mag, fs)
    sps      = round(fs / 1e6);
    pulse_w  = round(0.5 * sps);
    kern_len = round(8 * sps);
    kernel   = -ones(kern_len, 1);
    for t_us = [0, 1, 3.5, 4.5]
        t0 = round(t_us * sps) + 1;
        t1 = min(t0 + pulse_w - 1, kern_len);
        kernel(t0:t1) = 1;
    end
    [c, lags]           = xcorr(mag, kernel);
    [corr_peak, pk_idx] = max(c);
    snr_ratio           = corr_peak / (median(abs(c)) + eps);
    start_idx           = lags(pk_idx) + 1;
    data_start          = start_idx + kern_len;
    if data_start < 1, data_start = 1; end
end

function bits = adsb_decode_bits(mag, data_start, fs, n_bits)
    sps  = round(fs / 1e6);
    half = floor(sps / 2);
    bits = zeros(1, n_bits);
    for k = 0:n_bits-1
        i0 = data_start + k * sps;
        i1 = i0 + sps - 1;
        if i1 > length(mag), break; end
        chunk      = mag(i0:i1);
        bits(k+1)  = double(sum(chunk(1:half)) > sum(chunk(half+1:end)));
    end
end

function h = bits2hex(bits)
    chars = '0123456789ABCDEF';
    h = '';
    for i = 1:4:length(bits)-3
        v = bits(i)*8 + bits(i+1)*4 + bits(i+2)*2 + bits(i+3);
        h = [h, chars(v+1)];
    end
end
