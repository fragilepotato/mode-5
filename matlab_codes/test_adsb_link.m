%% ADS-B LINK TEST — Pluto TX -> ZedBoard RX (SMA Cable)
% Simple PPM ADS-B waveform, 5 cycles, 5s interval.
% Purpose: Verify the RF link works before adding Mode 5 crypto.
clear all; close all; clc;

% =====================================================================
%  CONFIGURATION
% =====================================================================
ip_pluto = 'ip:192.168.2.1';
ip_zed   = '192.168.1.10';

fc = 1090e6;                         % 1090 MHz (ADS-B frequency)
fs = 12e6;                           % 12 MSPS — ADS-B standard (12 samp/us = 6 per half-bit pulse)

MAX_CYCLES     = 5;
CYCLE_INTERVAL = 5;                  % seconds

% Gain table for auto-calibration [TX_pluto, RX_zed]
% At 12 MSPS we need higher TX drive to get enough amplitude
gain_table = [
%   TX_pluto  RX_zed
    -30       30
    -20       25
    -10       20
];

msg_hex = '8D4840D6202CC371C32CE0576098';   % Standard ADS-B test message

% =====================================================================
%  ENCODE ADS-B MESSAGE (PPM)
% =====================================================================
[tx_wave, tx_bits] = adsb_modulate(msg_hex, fs);

disp('========================================================');
disp('     ADS-B LINK TEST — Pluto TX -> ZedBoard RX');
disp('========================================================');
fprintf('Pluto  : %s\n', ip_pluto);
fprintf('ZedBoard: %s\n', ip_zed);
fprintf('Freq   : %.0f MHz   Fs: %.0f MSPS\n', fc/1e6, fs/1e6);
fprintf('Message: %s  (%d bits)\n', msg_hex, length(tx_bits));
disp('========================================================');

% =====================================================================
%  PHASE 0 — AUTO GAIN CALIBRATION
% =====================================================================
disp(' ');
disp('--- AUTO GAIN CALIBRATION ---');
best_gain_idx = 1;
best_max_amp  = 0;

for g = 1:size(gain_table, 1)
    tg = gain_table(g, 1);
    rg = gain_table(g, 2);
    fprintf('  Testing TX=%d dB  RX=%d dB ... ', tg, rg);

    try
        ptx = sdrtx('Pluto', 'RadioID', ip_pluto, ...
            'CenterFrequency', fc, 'BasebandSampleRate', fs);
        ptx.Gain = tg;

        zrx = sdrrx('AD936x', 'IPAddress', ip_zed, ...
            'CenterFrequency', fc, 'BasebandSampleRate', fs, ...
            'SamplesPerFrame', length(tx_wave)*4, ...
            'OutputDataType', 'double', 'ChannelMapping', 1);
        zrx.GainSource = 'Manual';
        zrx.Gain = rg;

        transmitRepeat(ptx, tx_wave);
        pause(1.0);           % 1 s settle — important at 12 MSPS
        for f = 1:3, zrx(); end   % flush
        release(ptx); release(zrx);

        mx = max(abs(cal_data));
        mn = mean(abs(cal_data));
        fprintf('max=%.4f  mean=%.4f', mx, mn);

        % Best = closest to 0.3 without clipping
        if mx > 0.05 && mx < 1.0 && mx > best_max_amp
            best_max_amp  = mx;
            best_gain_idx = g;
            fprintf('  <-- best so far');
        elseif mx >= 1.0
            fprintf('  CLIPPING');
        elseif mx <= 0.05
            fprintf('  TOO LOW');
        end
        fprintf('\n');
    catch ME
        fprintf('ERROR: %s\n', ME.message);
        try release(ptx); catch, end
        try release(zrx); catch, end
    end
end

tx_gain = gain_table(best_gain_idx, 1);
rx_gain = gain_table(best_gain_idx, 2);
fprintf('  >> Using TX=%d dB  RX=%d dB  (max_amp=%.4f)\n\n', ...
    tx_gain, rx_gain, best_max_amp);

if best_max_amp < 0.01
    error('No signal detected at any gain setting. Check SMA cable and power.');
end

% =====================================================================
%  MAIN TEST LOOP
% =====================================================================
fig = figure('Name', 'ADS-B Link Test', 'Position', [100 100 900 650]);
results = zeros(MAX_CYCLES, 3);  % [max_amp, BER, bit_errors]

for cycle = 1:MAX_CYCLES
    t_str = datestr(now, 'HH:MM:SS');
    fprintf('=== Cycle %d/%d  [%s] ===\n', cycle, MAX_CYCLES, t_str);

    try
        % --- TX (Pluto) ---
        ptx = sdrtx('Pluto', 'RadioID', ip_pluto, ...
            'CenterFrequency', fc, 'BasebandSampleRate', fs);
        ptx.Gain = tx_gain;

        % --- RX (ZedBoard) ---
        zrx = sdrrx('AD936x', 'IPAddress', ip_zed, ...
            'CenterFrequency', fc, 'BasebandSampleRate', fs, ...
            'SamplesPerFrame', length(tx_wave)*4, ...
            'OutputDataType', 'double', 'ChannelMapping', 1);
        zrx.GainSource = 'Manual';
        zrx.Gain = rx_gain;

        transmitRepeat(ptx, tx_wave);
        pause(1.0);                  % 1 s settle at 12 MSPS
        for f = 1:3, zrx(); end      % flush stale buffer
        data_rx = zrx();
        release(ptx); release(zrx);

        % --- SIGNAL STATS ---
        mag = abs(data_rx);
        mx = max(mag); mn = mean(mag); sd = std(mag);
        fprintf('  RX: max=%.4f  mean=%.4f  std=%.4f\n', mx, mn, sd);

        % --- DECODE ---
        [rx_bits, rx_hex, preamble_found, corr_peak] = adsb_decode(data_rx, fs);

        if preamble_found
            if length(rx_bits) >= length(tx_bits)
                rx_trimmed = rx_bits(1:length(tx_bits));
                bit_errs   = sum(abs(tx_bits - rx_trimmed));
                ber        = 100 * bit_errs / length(tx_bits);
            else
                bit_errs = -1;
                ber      = 100;
            end

            fprintf('  SENT:     %s\n', msg_hex);
            fprintf('  RECEIVED: %s\n', rx_hex);

            if bit_errs == 0
                fprintf('  RESULT: PERFECT MATCH\n');
            elseif bit_errs > 0
                fprintf('  RESULT: %d bit errors / %d  (BER=%.1f%%)\n', ...
                    bit_errs, length(tx_bits), ber);
                % Show first mismatches
                errs_idx = find(tx_bits ~= rx_trimmed, 10);
                fprintf('  First errors at bit positions: %s\n', num2str(errs_idx));
            else
                fprintf('  RESULT: Bit count mismatch (got %d, expected %d)\n', ...
                    length(rx_bits), length(tx_bits));
            end

            results(cycle, :) = [mx, ber, max(bit_errs, 0)];
        else
            fprintf('  RESULT: PREAMBLE NOT FOUND (corr_peak=%.4f)\n', corr_peak);
            results(cycle, :) = [mx, 100, length(tx_bits)];
        end

        % --- PLOT ---
        if ishandle(fig)
            figure(fig);

            subplot(3,2,1);
            plot(real(data_rx)); title(sprintf('RX I (Cycle %d)', cycle));
            xlabel('Sample'); ylabel('I'); grid on;

            subplot(3,2,2);
            plot(mag); hold on;
            yline(mx * 0.6, 'r--', 'Threshold'); hold off;
            title('RX Magnitude'); xlabel('Sample'); grid on;

            subplot(3,2,3);
            N = length(data_rx);
            f = (-N/2:N/2-1)*(fs/N);
            plot(f/1e6, 20*log10(abs(fftshift(fft(data_rx)))+eps));
            title('Spectrum'); xlabel('MHz'); ylabel('dB'); grid on;

            subplot(3,2,4);
            if preamble_found && length(rx_bits) >= 50
                stem(rx_bits(1:min(50,end)), 'filled', 'MarkerSize', 3);
                hold on;
                stem(tx_bits(1:min(50,end)), 'r.', 'MarkerSize', 6);
                hold off;
                legend('RX','TX','Location','best');
            end
            title('First 50 Bits (RX=blue, TX=red)'); ylim([-0.2 1.2]); grid on;

            subplot(3,2,[5 6]);
            bar(results(1:cycle, 2));
            xlabel('Cycle'); ylabel('BER (%)'); ylim([0 100]);
            title('Bit Error Rate per Cycle'); grid on;

            drawnow;
        end

    catch ME
        fprintf('  ERROR: %s\n', ME.message);
        try release(ptx); catch, end
        try release(zrx); catch, end
        results(cycle, :) = [0, 100, length(tx_bits)];
    end

    if cycle < MAX_CYCLES
        fprintf('  Waiting %d s ...\n\n', CYCLE_INTERVAL);
        pause(CYCLE_INTERVAL);
    end
end

% =====================================================================
%  SUMMARY
% =====================================================================
disp(' ');
disp('========================================================');
disp('                    TEST SUMMARY');
disp('========================================================');
fprintf('Cycles     : %d\n', MAX_CYCLES);
fprintf('TX Gain    : %d dB\n', tx_gain);
fprintf('RX Gain    : %d dB\n', rx_gain);
fprintf('Avg BER    : %.2f%%\n', mean(results(:,2)));
fprintf('Best BER   : %.2f%%  (Cycle %d)\n', min(results(:,2)), ...
    find(results(:,2) == min(results(:,2)), 1));
fprintf('Avg Max Amp: %.4f\n', mean(results(:,1)));
if all(results(:,2) == 0)
    disp('ALL CYCLES PERFECT — RF link is solid. Ready for Mode 5.');
elseif any(results(:,2) == 0)
    disp('SOME cycles perfect — link marginal. Adjust gains.');
else
    disp('NO perfect cycles — link NOT working. Debug hardware.');
end
disp('========================================================');

% =====================================================================
%  ADS-B MODULATOR (Hex -> PPM waveform)
% =====================================================================
function [waveform, bits] = adsb_modulate(hexStr, fs)
    bits = [];
    for i = 1:length(hexStr)
        val = hex2dec(hexStr(i));
        bits = [bits, bitget(val, 4:-1:1)];
    end

    sps = round(fs * 1e-6);  % samples per microsecond

    % Preamble (8 us): pulses at 0, 1, 3.5, 4.5 us
    preamble_len = round(8 * sps);
    preamble = zeros(1, preamble_len);
    pulse_len = round(0.5 * sps);
    for t = round([0, 1, 3.5, 4.5] * sps) + 1
        preamble(t : t + pulse_len - 1) = 1;
    end

    % Data: PPM (1 us per bit — first half=1 or second half=1)
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

    % Guard silence
    guard = zeros(1, sps * 20);
    sig = double([guard, preamble, data, guard]);

    % Force complex for Pluto SDR
    waveform = (sig.' + 1i*1e-12*ones(length(sig), 1));
    waveform = waveform / max(abs(waveform)) * 0.9;
end

% =====================================================================
%  ADS-B DECODER
% =====================================================================
function [bits, hex_str, found, peak_val] = adsb_decode(rx_data, fs)
    bits = []; hex_str = ''; found = false; peak_val = 0;

    sps     = round(fs / 1e6);       % 12 samples/us at 12 MSPS
    pulse_w = round(0.5 * sps);      % 6 samples per half-bit pulse

    % ----------------------------------------------------------------
    % DC removal via moving average (window = 20 us).
    % DO NOT use highpass() here — it destroys narrow PPM pulses by
    % ringing. Moving average subtraction removes only the slow AD9361
    % DC ramp without touching pulse structure.
    % ----------------------------------------------------------------
    mag = abs(rx_data(:));                        % envelope
    dc_window = round(20 * sps);                  % 20 us window
    mag_dc = movmean(mag, dc_window);             % slow DC baseline
    mag = mag - mag_dc;                           % remove baseline
    mag = max(mag, 0);                            % keep positive only
    mag = mag / (max(mag) + eps);                 % normalise 0->1

    % Build bipolar preamble kernel
    % ADS-B preamble: pulses at 0, 1, 3.5, 4.5 us (each 0.5 us wide)
    kern_len = round(8 * sps);
    kernel   = -ones(kern_len, 1);                % baseline = -1
    for t_us = [0, 1, 3.5, 4.5]
        t0 = round(t_us * sps) + 1;
        t1 = min(t0 + pulse_w - 1, kern_len);
        kernel(t0:t1) = 1;                        % pulse = +1
    end

    % Correlate
    [c, lags] = xcorr(mag, kernel);
    [peak_val, pk_idx] = max(c);
    noise     = median(abs(c));
    snr_ratio = peak_val / (noise + eps);

    fprintf('  [DECODE] corr_peak=%.3f  noise=%.3f  ratio=%.1f\n', ...
        peak_val, noise, snr_ratio);

    if snr_ratio < 4
        fprintf('  [DECODE] Preamble not found (need ratio >= 4)\n');
        return;
    end

    found = true;

    % xcorr lags are signed; convert to MATLAB 1-based sample index
    start_idx  = lags(pk_idx) + 1;
    data_start = start_idx + kern_len;

    fprintf('  [DECODE] Preamble at sample %d  data_start=%d\n', ...
        start_idx, data_start);

    if data_start < 1, data_start = 1; end

    % Decode 112 PPM bits
    bits = zeros(1, 112);
    for k = 0:111
        i0 = data_start + k * sps;
        i1 = i0 + sps - 1;
        if i1 > length(mag), break; end

        chunk = mag(i0:i1);
        half  = floor(sps / 2);

        % 1 = pulse in first half,  0 = pulse in second half
        if sum(chunk(1:half)) > sum(chunk(half+1:end))
            bits(k+1) = 1;
        else
            bits(k+1) = 0;
        end
    end

    % Bits -> Hex
    hex_str = '';
    chars   = '0123456789ABCDEF';
    for i = 1:4:112
        if i+3 > length(bits), break; end
        v = bits(i)*8 + bits(i+1)*4 + bits(i+2)*2 + bits(i+3);
        hex_str = [hex_str, chars(v+1)];
    end
end
