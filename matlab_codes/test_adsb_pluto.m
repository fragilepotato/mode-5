%% IFF GROUND STATION — ADALM Pluto SDR
%
% Implements the ground-station (interrogator) side of IFF:
%
%   1. TX interrogation on 1030 MHz  (challenge pulse sequence)
%   2. Switch to RX on 1090 MHz      (listen for aircraft reply)
%   3. Decode reply → extract aircraft ICAO address + squawk code
%   4. Report: "Aircraft identified" or "No reply"
%   5. Repeat every CYCLE_INTERVAL seconds
%
% Frequencies follow real-world IFF/SSR convention:
%   1030 MHz = Interrogation  (ground → aircraft)
%   1090 MHz = Reply          (aircraft → ground)
%
% Modulation: ADS-B PPM (pulse position modulation), 12 MSPS
%
% Run this FIRST.  Then run test_adsb_zedboard.m (aircraft transponder).
clear all; close all; clc;

% =====================================================================
%  CONFIGURATION
% =====================================================================
ip_pluto       = 'ip:192.168.2.1';
fc_interrogate = 1030e6;   % Ground → Aircraft
fc_reply       = 1090e6;   % Aircraft → Ground
fs             = 12e6;     % 12 MSPS
tx_gain        = -20;      % dB  (SMA cable)
rx_gain        = 10;       % dB

TX_HOLD        = 3.0;      % seconds to hold interrogation TX
RX_LISTEN      = 6.0;      % seconds to listen for reply
MAX_CYCLES     = 5;
CYCLE_INTERVAL = 5;        % seconds between interrogations

% Interrogation message (ground station ID + mode request)
% Format: [DF=11 (8 bits)][InterrogID 1A2B (16 bits)][ModeCode 05 (8 bits)]
%         [Challenge nonce (32 bits)][Parity/CRC (24 bits)]  = 88 bits
% Encoded as 22 hex chars.
%
% For this test we use a fixed interrogation so the transponder can
% recognise it by the DF=11 header and reply with its identity.
INTERROG_DF     = 'B0';          % Downlink Format 11 = All-call interrogation
INTERROG_ID     = '1A2B';        % Ground station ID
MODE_CODE       = '05';          % Mode 5 request
CHALLENGE_HEX   = 'D2CE21DA';   % Random challenge nonce

% =====================================================================
%  BUILD INTERROGATION WAVEFORM
% =====================================================================
interrog_hex = [INTERROG_DF, INTERROG_ID, MODE_CODE, CHALLENGE_HEX];
% Append CRC-24 placeholder (all-zero — transponder ignores for now)
interrog_hex = [interrog_hex, '000000'];    % 22 hex = 88 bits total

[tx_wave, tx_bits] = adsb_modulate(interrog_hex, fs);

% Reply capture buffer (aircraft reply = 112 bits = 28 hex ADS-B)
reply_frame_samples = round((20 + 8 + 112 + 20) * fs * 1e-6) * 4;

fprintf('========================================================\n');
fprintf('   IFF GROUND STATION — ADALM Pluto\n');
fprintf('========================================================\n');
fprintf('Station     : %s   (ID: %s)\n', ip_pluto, INTERROG_ID);
fprintf('Interrogate : %.0f MHz → TX\n', fc_interrogate/1e6);
fprintf('Listen Reply: %.0f MHz → RX\n', fc_reply/1e6);
fprintf('Fs          : %.0f MSPS\n', fs/1e6);
fprintf('Challenge   : %s\n', CHALLENGE_HEX);
fprintf('TX hold     : %.1f s    RX listen: %.1f s\n', TX_HOLD, RX_LISTEN);
fprintf('Cycles      : %d  |  Interval: %d s\n', MAX_CYCLES, CYCLE_INTERVAL);
fprintf('========================================================\n\n');

% =====================================================================
%  MAIN INTERROGATION LOOP
% =====================================================================
fig = figure('Name','IFF Ground Station','Position',[100 100 900 620]);
results = zeros(MAX_CYCLES, 4);  % [reply_found, bit_errors, snr, max_amp]

for cycle = 1:MAX_CYCLES
    fprintf('=== Interrogation %d/%d  [%s] ===\n', ...
        cycle, MAX_CYCLES, datestr(now,'HH:MM:SS'));

    reply_iq   = [];
    reply_found = false;
    rx_hex      = '';
    snr_reply   = 0;

    % ------------------------------------------------------------------
    %  STEP 1 — TX interrogation on 1030 MHz
    % ------------------------------------------------------------------
    fprintf('  [TX] Interrogation on %.0f MHz for %.1f s ...\n', ...
        fc_interrogate/1e6, TX_HOLD);
    try
        ptx = sdrtx('Pluto', 'RadioID', ip_pluto, ...
            'CenterFrequency', fc_interrogate, 'BasebandSampleRate', fs);
        ptx.Gain = tx_gain;
        transmitRepeat(ptx, tx_wave);
        pause(TX_HOLD);
        release(ptx);
        fprintf('  [TX] Interrogation sent.\n');
    catch ME
        fprintf('  [TX ERROR] %s\n', ME.message);
        try, release(ptx); catch, end
        results(cycle,:) = [0, 0, 0, 0];
        if cycle < MAX_CYCLES, pause(CYCLE_INTERVAL); end
        continue;
    end

    % ------------------------------------------------------------------
    %  STEP 2 — Listen for aircraft reply on 1090 MHz
    % ------------------------------------------------------------------
    fprintf('  [RX] Listening for reply on %.0f MHz (%.1f s) ...\n', ...
        fc_reply/1e6, RX_LISTEN);
    try
        prx = sdrrx('Pluto', 'RadioID', ip_pluto, ...
            'CenterFrequency', fc_reply, 'BasebandSampleRate', fs, ...
            'SamplesPerFrame', reply_frame_samples, ...
            'OutputDataType', 'double');
        prx.GainSource = 'Manual';
        prx.Gain = rx_gain;

        pause(1.0);    % settle
        t_deadline = tic;
        while toc(t_deadline) < (RX_LISTEN - 1.0)
            reply_iq = prx();
            mx = max(abs(reply_iq));
            if mx < 0.02, continue; end   % no signal yet

            mag = adsb_preprocess(reply_iq, fs);
            [data_start, ~, snr_ratio] = adsb_framesync(mag, fs);

            if snr_ratio >= 4
                reply_bits = adsb_decode_bits(mag, data_start, fs, 112);
                rx_hex     = bits2hex(reply_bits);
                snr_reply  = snr_ratio;
                reply_found = true;
                break;
            end
        end
        release(prx);
    catch ME
        fprintf('  [RX ERROR] %s\n', ME.message);
        try, release(prx); catch, end
    end

    % ------------------------------------------------------------------
    %  STEP 3 — Identify the aircraft from the reply
    % ------------------------------------------------------------------
    if reply_found
        % ADS-B reply format: DF(8) + ICAO(24) + ME(56) + PI(24) = 112 bits
        %   ICAO address is bits 9-32 (hex chars 3-8)
        %   Squawk/IFF code is in ME field chars 9-22
        icao_hex      = rx_hex(3:8);
        iff_data      = rx_hex(9:22);

        fprintf('  ┌──────────────────────────────────────────┐\n');
        fprintf('  │  AIRCRAFT REPLY RECEIVED                 │\n');
        fprintf('  │  Full frame : %s  │\n', rx_hex);
        fprintf('  │  ICAO addr  : %s                         │\n', icao_hex);
        fprintf('  │  IFF data   : %s              │\n', iff_data);
        fprintf('  │  SNR ratio  : %.1f                        │\n', snr_reply);
        fprintf('  │  STATUS     : AIRCRAFT IDENTIFIED         │\n');
        fprintf('  └──────────────────────────────────────────┘\n');
        results(cycle,:) = [1, 0, snr_reply, mx];
    else
        fprintf('  ┌──────────────────────────────────────────┐\n');
        fprintf('  │  NO REPLY — Aircraft not responding       │\n');
        fprintf('  └──────────────────────────────────────────┘\n');
        results(cycle,:) = [0, 0, 0, 0];
    end

    % ------------------------------------------------------------------
    %  Live plot
    % ------------------------------------------------------------------
    if ishandle(fig)
        figure(fig);
        subplot(3,2,1); plot(real(tx_wave));
        title('Interrogation Waveform (TX)');
        xlabel('Sample'); ylabel('I'); grid on;

        subplot(3,2,2);
        if ~isempty(reply_iq), plot(real(reply_iq)); end
        title(sprintf('Reply RX — Cycle %d', cycle));
        xlabel('Sample'); ylabel('I'); grid on;

        subplot(3,2,3);
        if ~isempty(reply_iq)
            N = length(reply_iq); f = (-N/2:N/2-1)*(fs/N);
            plot(f/1e6, 20*log10(abs(fftshift(fft(reply_iq)))+eps));
            xlabel('MHz'); ylabel('dB'); grid on;
        end
        title('Reply Spectrum');

        subplot(3,2,4);
        if reply_found
            stem(reply_bits(1:min(56,end)),'filled','MarkerSize',3);
            title(sprintf('Reply bits — ICAO %s', icao_hex));
        end
        ylim([-0.2 1.2]); grid on;

        subplot(3,2,[5 6]); cla; axis off;
        if reply_found
            set(gca,'Color',[0.2 0.8 0.2]);
            text(0.5, 0.5, sprintf('Cycle %d — AIRCRAFT %s IDENTIFIED\nIFF: %s   SNR: %.0f', ...
                cycle, icao_hex, iff_data, snr_reply), ...
                'FontSize', 14, 'FontWeight', 'bold', ...
                'HorizontalAlignment','center','VerticalAlignment','middle');
        else
            set(gca,'Color',[0.8 0.2 0.2]);
            text(0.5, 0.5, sprintf('Cycle %d — NO REPLY', cycle), ...
                'FontSize', 14, 'FontWeight', 'bold', ...
                'HorizontalAlignment','center','VerticalAlignment','middle');
        end
        drawnow;
    end

    if cycle < MAX_CYCLES
        fprintf('  Next interrogation in %d s ...\n\n', CYCLE_INTERVAL);
        pause(CYCLE_INTERVAL);
    end
end

% =====================================================================
%  SUMMARY
% =====================================================================
n_replies = sum(results(:,1));
fprintf('\n========================================================\n');
fprintf('         GROUND STATION INTERROGATION SUMMARY\n');
fprintf('========================================================\n');
fprintf('Interrogations sent : %d\n', MAX_CYCLES);
fprintf('Replies received    : %d/%d\n', n_replies, MAX_CYCLES);
if n_replies == MAX_CYCLES
    fprintf('ALL INTERROGATIONS ANSWERED — Aircraft identification confirmed.\n');
elseif n_replies > 0
    fprintf('Partial replies — intermittent link.\n');
else
    fprintf('No replies — check transponder is running.\n');
end
fprintf('========================================================\n');

% =====================================================================
%  ADS-B PPM MODULATOR
% =====================================================================
function [waveform, bits] = adsb_modulate(hexStr, fs)
    bits = [];
    for i = 1:length(hexStr)
        val  = hex2dec(hexStr(i));
        bits = [bits, bitget(val, 4:-1:1)];
    end
    sps          = round(fs * 1e-6);
    preamble_len = round(8 * sps);
    preamble     = zeros(1, preamble_len);
    pulse_len    = round(0.5 * sps);
    for t_us = [0, 1, 3.5, 4.5]
        t0 = round(t_us * sps) + 1;
        t1 = min(t0 + pulse_len - 1, preamble_len);
        preamble(t0:t1) = 1;
    end
    data = [];
    for b = bits
        pat = zeros(1, sps); mid = floor(sps/2);
        if b == 1, pat(1:mid) = 1; else, pat(mid+1:end) = 1; end
        data = [data, pat];
    end
    guard = zeros(1, sps * 20);
    sig   = double([guard, preamble, data, guard]);
    sig   = sig / max(abs(sig)) * 0.9;
    waveform = sig.' + 1i * 1e-12 * ones(length(sig), 1);
end

% =====================================================================
%  SHARED DSP  (same 3 IP-core functions as ZedBoard)
% =====================================================================
function mag = adsb_preprocess(rx_iq, fs)
    sps = round(fs / 1e6);
    mag = abs(rx_iq(:));
    mag = mag - movmean(mag, round(20 * sps));
    mag = max(mag, 0);
    mag = mag / (max(mag) + eps);
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
    data_start          = lags(pk_idx) + 1 + kern_len;
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
        chunk     = mag(i0:i1);
        bits(k+1) = double(sum(chunk(1:half)) > sum(chunk(half+1:end)));
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
