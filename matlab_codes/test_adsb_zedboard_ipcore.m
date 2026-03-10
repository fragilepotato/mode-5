%% IFF TRANSPONDER — IP Core Ready (ZedBoard FMCOMMS3 / AD9361)
%
% Stripped version of test_adsb_zedboard.m for HDL Coder IP core generation.
%   - No plots, no figure, no debug prints
%   - Minimal control flow: listen -> decode -> reply -> repeat
%   - Three IP core candidate functions unchanged from prototype
%   - Button trigger via GPIO stub (maps to physical pushbutton)
%   - LED indicator via GPIO stub (maps to physical LEDs)
%
% Target: MATLAB HDL Coder -> Vivado 2023.2
%
% HDL-ready functions:
%   adsb_preprocess   — envelope extraction + DC removal
%   adsb_framesync    — preamble matched filter
%   adsb_decode_bits  — PPM bit decisions
%
% GPIO mapping (directly wired in Vivado block design):
%   BTN_CENTER  -> start 5-cycle burst
%   LED0        -> RX active (listening on 1030 MHz)
%   LED1        -> TX active (transmitting on 1090 MHz)
%   LED2        -> valid interrogation decoded this cycle
clear all; close all; clc;

% =====================================================================
%  CONFIGURATION
% =====================================================================
ip_zed          = '192.168.1.10';
fc_interrogate  = 1030e6;
fc_reply        = 1090e6;
fs              = 12e6;
rx_gain         = 25;
tx_gain         = -20;

REPLY_HOLD      = 4.0;
LISTEN_TIMEOUT  = 15;
CYCLES_PER_BURST = 5;

% Aircraft identity
ICAO_HEX        = '4840D6';
SQUAWK_HEX      = '2CC371C32CE0';

interrog_frame_samples = round((20 + 8 + 88 + 20) * fs * 1e-6) * 4;
INTERROG_SYNC   = 'B0';

% =====================================================================
%  GPIO — Stubs (replace with AXI GPIO read/write for IP core)
%
%  For Vivado block design:
%    - Add AXI GPIO IP core, connect to BTN and LED ports
%    - These stubs will be replaced by register read/write
% =====================================================================
LED_RX  = 0;   % LED0
LED_TX  = 1;   % LED1
LED_OK  = 2;   % LED2

% =====================================================================
%  MAIN TRANSPONDER LOOP
% =====================================================================
while true
    % --- Wait for button trigger ---
    gpio_write_led(LED_RX, 0);
    gpio_write_led(LED_TX, 0);
    gpio_write_led(LED_OK, 0);

    fprintf('Waiting for trigger ...\n');
    wait_for_button();

    % --- Run burst ---
    for cycle = 1:CYCLES_PER_BURST

        interrog_ok = false;
        interrog_hex = '';

        % =============================================================
        %  RX — Listen on 1030 MHz
        % =============================================================
        gpio_write_led(LED_RX, 1);

        try
            zrx = sdrrx('AD936x', 'IPAddress', ip_zed, ...
                'CenterFrequency', fc_interrogate, ...
                'BasebandSampleRate', fs, ...
                'SamplesPerFrame', interrog_frame_samples, ...
                'OutputDataType', 'double', 'ChannelMapping', 1);
            zrx.GainSource = 'Manual';
            zrx.Gain       = rx_gain;

            pause(1.0);
            t_start = tic;
            while toc(t_start) < LISTEN_TIMEOUT
                rx_iq = zrx();
                if max(abs(rx_iq)) < 0.02, continue; end

                mag = adsb_preprocess(rx_iq, fs);
                [data_start, ~, snr_ratio] = adsb_framesync(mag, fs);

                if snr_ratio >= 4
                    rx_bits      = adsb_decode_bits(mag, data_start, fs, 88);
                    interrog_hex = bits2hex(rx_bits);

                    if strncmp(interrog_hex, INTERROG_SYNC, 2)
                        interrog_ok = true;
                        challenge   = interrog_hex(9:16);
                        break;
                    end
                end
            end
            release(zrx);
        catch
            try, release(zrx); catch, end %#ok<NOCOM>
        end

        gpio_write_led(LED_RX, 0);

        % =============================================================
        %  TX — Reply on 1090 MHz
        % =============================================================
        if interrog_ok
            gpio_write_led(LED_OK, 1);
            gpio_write_led(LED_TX, 1);

            reply_hex = ['8D', ICAO_HEX, SQUAWK_HEX, challenge(1:6)];

            try
                [tx_wave, ~] = adsb_modulate(reply_hex, fs);
                ztx = sdrtx('AD936x', 'IPAddress', ip_zed, ...
                    'CenterFrequency', fc_reply, ...
                    'BasebandSampleRate', fs, ...
                    'ChannelMapping', 1);
                ztx.Gain = tx_gain;

                transmitRepeat(ztx, tx_wave);
                pause(REPLY_HOLD);
                release(ztx);
            catch
                try, release(ztx); catch, end %#ok<NOCOM>
            end

            gpio_write_led(LED_TX, 0);
        end

        gpio_write_led(LED_OK, 0);
        pause(0.3);
    end
end


% #####################################################################
%  GPIO STUBS — Replace with AXI GPIO register access for IP core
% #####################################################################

function wait_for_button()
    % STUB: In IP core, read AXI GPIO register for BTN_CENTER.
    % For MATLAB prototype: wait for keyboard press.
    input('  >> Press ENTER to start burst ... ', 's');
end

function gpio_write_led(led_id, value)
    % STUB: In IP core, write AXI GPIO register.
    % For MATLAB prototype: print to console.
    names = {'LED0-RX', 'LED1-TX', 'LED2-OK'};
    if value
        fprintf('  [GPIO] %s = ON\n', names{led_id+1});
    end
end


% #####################################################################
%  IP CORE CANDIDATES — Identical to prototype (no changes needed)
% #####################################################################

% =====================================================================
%  Stage 1: Envelope + DC removal
%  HDL: sliding-window accumulator + max tracker
% =====================================================================
function mag = adsb_preprocess(rx_iq, fs)
    sps    = round(fs / 1e6);
    mag    = abs(rx_iq(:));
    dc_win = round(20 * sps);
    mag    = mag - movmean(mag, dc_win);
    mag    = max(mag, 0);
    mag    = mag / (max(mag) + eps);
end

% =====================================================================
%  Stage 2: Preamble matched filter / frame sync
%  HDL: FIR with 96-tap ROM coefficients
% =====================================================================
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

% =====================================================================
%  Stage 3: PPM bit decisions
%  HDL: adder + comparator (no divider)
% =====================================================================
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

% =====================================================================
%  HELPERS
% =====================================================================
function [waveform, bits] = adsb_modulate(hexStr, fs)
    bits = [];
    for i = 1:length(hexStr)
        bits = [bits, bitget(hex2dec(hexStr(i)), 4:-1:1)]; %#ok<AGROW>
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
        data = [data, pat]; %#ok<AGROW>
    end
    guard    = zeros(1, sps * 20);
    sig      = double([guard, preamble, data, guard]);
    sig      = sig / max(abs(sig)) * 0.9;
    waveform = sig.' + 1i * 1e-12 * ones(length(sig), 1);
end

function h = bits2hex(bits)
    chars = '0123456789ABCDEF';
    h = '';
    for i = 1:4:length(bits)-3
        v = bits(i)*8 + bits(i+1)*4 + bits(i+2)*2 + bits(i+3);
        h = [h, chars(v+1)]; %#ok<AGROW>
    end
end
