%% ADS-B RX — ZedBoard FMCOMMS3 / AD9361
%
% Receives and decodes ADS-B PPM waveform from Pluto on 1090 MHz.
%
% Signal processing is factored into three standalone functions:
%   adsb_preprocess()  — DC removal, envelope normalisation
%   adsb_framesync()   — preamble correlator (find frame start)
%   adsb_decode_bits() — PPM bit decisions
%
% These three functions are the IP core candidates for Vivado 2023.2.
% Their I/O is kept to fixed-size arrays with no persistent state so
% they can be directly targeted by MATLAB HDL Coder.
%
% Run AFTER test_adsb_pluto.m has started transmitting.
clear all; close all; clc;

% =====================================================================
%  CONFIGURATION
% =====================================================================
ip_zed   = '192.168.1.10';
fc       = 1090e6;    % ADS-B frequency
fs       = 12e6;      % 12 MSPS
rx_gain  = 25;        % dB  (SMA cable, adjust if signal too weak/strong)

MAX_CYCLES     = 5;
CYCLE_INTERVAL = 5;   % seconds between captures

% Reference message to compute BER against
msg_hex  = '8D4840D6202CC371C32CE0576098';
tx_bits  = hex2bits(msg_hex);

% Capture buffer = 4× one ADS-B frame
%   frame = (20+8+112+20) µs guard+preamble+data+guard ≈ 160 µs at 12 MSPS
frame_samples = round((20 + 8 + 112 + 20) * fs * 1e-6) * 4;

fprintf('========================================================\n');
fprintf('   ADS-B RX — ZedBoard FMCOMMS3\n');
fprintf('========================================================\n');
fprintf('ZedBoard: %s\n', ip_zed);
fprintf('Freq    : %.0f MHz   Fs: %.0f MSPS\n', fc/1e6, fs/1e6);
fprintf('RX Gain : %d dB\n', rx_gain);
fprintf('Buffer  : %d samples  (%.2f ms)\n', ...
    frame_samples, frame_samples/fs*1e3);
fprintf('========================================================\n\n');

% =====================================================================
%  MAIN CAPTURE LOOP
% =====================================================================
fig     = figure('Name','ADS-B RX — ZedBoard','Position',[100 100 920 660]);
results = zeros(MAX_CYCLES, 3);   % [max_amp, bit_errors, found]

for cycle = 1:MAX_CYCLES
    fprintf('=== Cycle %d/%d  [%s] ===\n', cycle, MAX_CYCLES, datestr(now,'HH:MM:SS'));

    try
        % ----- Hardware setup -----
        zrx = sdrrx('AD936x', 'IPAddress', ip_zed, ...
            'CenterFrequency', fc, 'BasebandSampleRate', fs, ...
            'SamplesPerFrame', frame_samples, ...
            'OutputDataType', 'double', 'ChannelMapping', 1);
        zrx.GainSource = 'Manual';
        zrx.Gain       = rx_gain;

        pause(1.0);                   % settle — important at 12 MSPS
        for k = 1:3, zrx(); end       % flush stale buffer
        rx_iq = zrx();
        release(zrx);

        % ----- Signal stats -----
        mx = max(abs(rx_iq));
        mn = mean(abs(rx_iq));
        fprintf('  RX: samples=%d  max=%.4f  mean=%.4f\n', ...
            length(rx_iq), mx, mn);

        if mx > 1.2
            fprintf('  WARNING: ADC clipping (max > 1.2) — reduce rx_gain.\n');
        elseif mx < 0.02
            fprintf('  WARNING: Signal too weak (max < 0.02) — increase rx_gain or check cable.\n');
        end

        % ====================================================
        %  IP CORE CANDIDATE FUNCTIONS
        %  (fixed-size, no persistent state, HDL Coder ready)
        % ====================================================

        % Stage 1 — DC removal + normalisation
        mag = adsb_preprocess(rx_iq, fs);

        % Stage 2 — Preamble correlation → frame start index
        [data_start, corr_peak, snr_ratio] = adsb_framesync(mag, fs);
        fprintf('  [SYNC] corr_peak=%.3f  SNR_ratio=%.1f  data_start=%d\n', ...
            corr_peak, snr_ratio, data_start);

        % Stage 3 — PPM bit decisions
        if snr_ratio >= 4 && data_start > 0
            rx_bits = adsb_decode_bits(mag, data_start, fs, 112);
            found   = true;
        else
            rx_bits = zeros(1, 112);
            found   = false;
        end

        % ====================================================
        %  RESULTS
        % ====================================================
        if found
            rx_hex   = bits2hex(rx_bits);
            bit_errs = sum(rx_bits(1:length(tx_bits)) ~= tx_bits);
            fprintf('  SENT    : %s\n', msg_hex);
            fprintf('  RECEIVED: %s\n', rx_hex);
            if bit_errs == 0
                fprintf('  RESULT  : PERFECT MATCH\n');
            else
                fprintf('  RESULT  : %d/%d bit errors\n', bit_errs, length(tx_bits));
                bad = find(rx_bits(1:length(tx_bits)) ~= tx_bits, 10);
                fprintf('  Errors at bits: %s\n', num2str(bad));
            end
            results(cycle,:) = [mx, bit_errs, 1];
        else
            fprintf('  RESULT  : PREAMBLE NOT FOUND — check Pluto is transmitting.\n');
            results(cycle,:) = [mx, length(tx_bits), 0];
        end

        % ----- Live plot -----
        if ishandle(fig)
            figure(fig);
            subplot(3,2,1); plot(real(rx_iq));
            title(sprintf('RX I  (Cycle %d)', cycle));
            xlabel('Sample'); ylabel('I'); grid on;

            subplot(3,2,2); plot(mag);
            title('Envelope after DC removal');
            xlabel('Sample'); grid on;
            if found && data_start > 0
                hold on; xline(data_start,'r--','Data start'); hold off;
            end

            subplot(3,2,3);
            N = length(rx_iq);
            f = (-N/2:N/2-1)*(fs/N);
            plot(f/1e6, 20*log10(abs(fftshift(fft(rx_iq)))+eps));
            title('Spectrum'); xlabel('MHz'); ylabel('dB'); grid on;

            subplot(3,2,4);
            if found
                stem(rx_bits(1:min(56,end)),'filled','MarkerSize',3);
                hold on;
                stem(tx_bits(1:min(56,end)),'r.','MarkerSize',6);
                hold off; legend('RX','TX','Location','best');
            end
            title('First 56 bits (RX=blue, TX=red)');
            ylim([-0.2 1.2]); grid on;

            subplot(3,2,[5 6]);
            bar(results(1:cycle,2));
            xlabel('Cycle'); ylabel('Bit Errors'); ylim([0 112+5]);
            title('Bit Errors per Cycle'); grid on;
            drawnow;
        end

    catch ME
        fprintf('  ERROR: %s\n', ME.message);
        try, release(zrx); catch, end
        results(cycle,:) = [0, 112, 0];
    end

    if cycle < MAX_CYCLES
        fprintf('  Waiting %d s ...\n\n', CYCLE_INTERVAL);
        pause(CYCLE_INTERVAL);
    end
end

% =====================================================================
%  SUMMARY
% =====================================================================
fprintf('\n========================================================\n');
fprintf('                    TEST SUMMARY\n');
fprintf('========================================================\n');
fprintf('Cycles    : %d\n', MAX_CYCLES);
fprintf('RX Gain   : %d dB\n', rx_gain);
fprintf('Avg Errors: %.1f bits/cycle\n', mean(results(:,2)));
fprintf('Found     : %d/%d cycles\n', sum(results(:,3)), MAX_CYCLES);
if all(results(:,2) == 0 & results(:,3) == 1)
    fprintf('ALL CYCLES PERFECT — ready to add Mode 5 logic.\n');
elseif any(results(:,2) == 0 & results(:,3) == 1)
    fprintf('SOME cycles perfect — link is marginal.\n');
else
    fprintf('NO perfect cycles — check RF link and gains.\n');
end
fprintf('========================================================\n');

% =====================================================================
%  IP CORE CANDIDATE — Stage 1
%  adsb_preprocess: Envelope extraction + moving-average DC removal
%
%  IN:  rx_iq  — complex baseband I/Q vector (double)
%       fs     — sample rate (Hz)
%  OUT: mag    — normalised envelope, real, range [0..1]
%
%  HDL notes: movmean becomes a sliding-window accumulator.
%             Division by max is a single multiplier after max-tracking.
% =====================================================================
function mag = adsb_preprocess(rx_iq, fs)
    sps    = round(fs / 1e6);           % 12 samples/µs at 12 MSPS
    mag    = abs(rx_iq(:));             % envelope (rectifier)
    dc_win = round(20 * sps);           % 20 µs window = 240 samples
    mag    = mag - movmean(mag, dc_win);% subtract slow DC baseline
    mag    = max(mag, 0);               % half-wave rectify
    mag    = mag / (max(mag) + eps);    % normalise 0→1
end

% =====================================================================
%  IP CORE CANDIDATE — Stage 2
%  adsb_framesync: Bipolar preamble correlator
%
%  IN:  mag    — normalised envelope from adsb_preprocess
%       fs     — sample rate (Hz)
%  OUT: data_start — 1-based sample index where data bits begin
%       corr_peak  — absolute correlation peak value
%       snr_ratio  — peak / median (quality indicator, threshold = 4)
%
%  HDL notes: xcorr is a matched filter (FIR with reversed coefficients).
%             The 96-tap kernel is fixed at compile time → ROM coefficients.
% =====================================================================
function [data_start, corr_peak, snr_ratio] = adsb_framesync(mag, fs)
    sps      = round(fs / 1e6);
    pulse_w  = round(0.5 * sps);

    % Build bipolar preamble kernel (96 samples at 12 MSPS for 8 µs preamble)
    kern_len = round(8 * sps);
    kernel   = -ones(kern_len, 1);
    for t_us = [0, 1, 3.5, 4.5]
        t0 = round(t_us * sps) + 1;
        t1 = min(t0 + pulse_w - 1, kern_len);
        kernel(t0:t1) = 1;
    end

    [c, lags]          = xcorr(mag, kernel);
    [corr_peak, pk_idx] = max(c);
    snr_ratio           = corr_peak / (median(abs(c)) + eps);

    start_idx  = lags(pk_idx) + 1;
    data_start = start_idx + kern_len;
    if data_start < 1, data_start = 1; end
end

% =====================================================================
%  IP CORE CANDIDATE — Stage 3
%  adsb_decode_bits: PPM symbol decisions
%
%  IN:  mag        — normalised envelope from adsb_preprocess
%       data_start — from adsb_framesync
%       fs         — sample rate (Hz)
%       n_bits     — number of PPM bits to decode (112 for ADS-B)
%  OUT: bits       — decoded bit vector (0/1)
%
%  HDL notes: Each bit = compare sum(first-half-samples) vs
%             sum(second-half-samples) within one symbol period.
%             Pure accumulate-and-compare, no division needed.
% =====================================================================
function bits = adsb_decode_bits(mag, data_start, fs, n_bits)
    sps  = round(fs / 1e6);
    half = floor(sps / 2);
    bits = zeros(1, n_bits);

    for k = 0:n_bits-1
        i0 = data_start + k * sps;
        i1 = i0 + sps - 1;
        if i1 > length(mag), break; end

        chunk = mag(i0:i1);
        % bit=1: energy in first half,  bit=0: energy in second half
        bits(k+1) = double(sum(chunk(1:half)) > sum(chunk(half+1:end)));
    end
end

% =====================================================================
%  HELPER — hex string -> bit vector
% =====================================================================
function bits = hex2bits(hexStr)
    bits = [];
    for i = 1:length(hexStr)
        val  = hex2dec(hexStr(i));
        bits = [bits, bitget(val, 4:-1:1)];
    end
end

% =====================================================================
%  HELPER — bit vector -> hex string
% =====================================================================
function h = bits2hex(bits)
    chars = '0123456789ABCDEF';
    h = '';
    for i = 1:4:length(bits)-3
        v = bits(i)*8 + bits(i+1)*4 + bits(i+2)*2 + bits(i+3);
        h = [h, chars(v+1)];
    end
end
