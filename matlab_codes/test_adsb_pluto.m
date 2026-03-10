%% ADS-B TX — ADALM Pluto SDR (Interrogator side)
%
% Continuously transmits an ADS-B PPM waveform on 1090 MHz.
% Run this script FIRST, then run test_adsb_zedboard.m to receive.
%
% Usage:
%   1. Open this file in MATLAB and press Run.
%   2. Wait for "Transmitting — press any key to stop." message.
%   3. In a separate MATLAB window run test_adsb_zedboard.m.
%   4. Press any key here (or Ctrl-C) to stop transmission.
clear all; clc;

% =====================================================================
%  CONFIGURATION
% =====================================================================
ip_pluto  = 'ip:192.168.2.1';
fc        = 1090e6;    % ADS-B frequency
fs        = 12e6;      % 12 MSPS — ADS-B standard rate
tx_gain   = -20;       % dB  (SMA cable, target RX max ~0.3)

% Standard ADS-B test message (Extended Squitter, Airborne Position)
msg_hex = '8D4840D6202CC371C32CE0576098';

% =====================================================================
%  BUILD WAVEFORM
% =====================================================================
[tx_wave, tx_bits] = adsb_modulate(msg_hex, fs);

fprintf('========================================================\n');
fprintf('   ADS-B TX — ADALM Pluto\n');
fprintf('========================================================\n');
fprintf('Pluto   : %s\n', ip_pluto);
fprintf('Freq    : %.0f MHz   Fs: %.0f MSPS\n', fc/1e6, fs/1e6);
fprintf('TX Gain : %d dB\n', tx_gain);
fprintf('Message : %s  (%d bits)\n', msg_hex, length(tx_bits));
fprintf('Frame   : %d samples  (%.2f ms)\n', ...
    length(tx_wave), length(tx_wave)/fs*1e3);
fprintf('========================================================\n\n');

% =====================================================================
%  START TRANSMISSION
% =====================================================================
ptx = sdrtx('Pluto', 'RadioID', ip_pluto, ...
    'CenterFrequency', fc, 'BasebandSampleRate', fs);
ptx.Gain = tx_gain;

transmitRepeat(ptx, tx_wave);
fprintf('>> Transmitting — press any key to stop.\n');
pause;    % hold until keypress or Ctrl-C

release(ptx);
fprintf('>> Transmission stopped.\n');

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
