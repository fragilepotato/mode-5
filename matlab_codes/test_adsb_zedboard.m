%% IFF AIRCRAFT TRANSPONDER — ZedBoard FMCOMMS3 / AD9361
%
% Implements the aircraft (transponder) side of IFF:
%   1. Listen on 1030 MHz for ground station interrogation
%   2. Decode interrogation -> validate sync header (DF=B0)
%   3. If valid: TX reply on 1090 MHz containing aircraft identity
%   4. Go back to listening
%
% Operation modes:
%   'continuous'  — loops indefinitely, no trigger needed
%   'triggered'   — waits for START button press, then runs
%                    CYCLES_PER_BURST cycles before waiting again
%
% Indicator panel (figure window):
%   BLUE   = Listening / receiving on 1030 MHz
%   GREEN  = Transmitting reply on 1090 MHz
%   GRAY   = Idle / waiting for trigger
%   (For IP core deployment: these map to physical ZedBoard LEDs)
%   (For IP core deployment: START button maps to physical pushbutton)
%
% Frequencies (real-world IFF/SSR convention):
%   1030 MHz = Interrogation  (ground -> aircraft) — this side RX
%   1090 MHz = Reply          (aircraft -> ground) — this side TX
%
% IP core candidate functions (adsb_preprocess, adsb_framesync,
% adsb_decode_bits) have fixed I/O, no persistent state, and are ready
% for MATLAB HDL Coder -> Vivado 2023.2.
clear all; close all; clc;

% =====================================================================
%  CONFIGURATION
% =====================================================================
ip_zed          = '192.168.1.10';
fc_interrogate  = 1030e6;       % RX: listen for interrogation
fc_reply        = 1090e6;       % TX: send aircraft identity reply
fs              = 12e6;         % 12 MSPS
rx_gain         = 25;           % dB  (adjust for SMA cable)
tx_gain         = -20;          % dB

REPLY_HOLD      = 4.0;         % seconds to hold reply TX on air
LISTEN_TIMEOUT  = 15;           % seconds to listen before giving up
CYCLES_PER_BURST = 5;           % cycles per button press (triggered mode)
OPERATION_MODE  = 'triggered';  % 'continuous' or 'triggered'

% --- Aircraft identity (what the transponder reports) ---
ICAO_HEX        = '4840D6';        % 24-bit ICAO address (this aircraft)
SQUAWK_HEX      = '2CC371C32CE0';  % Squawk/IFF + encoded altitude + status

% Capture buffer
interrog_frame_samples = round((20 + 8 + 88 + 20) * fs * 1e-6) * 4;
INTERROG_SYNC = 'B0';

% =====================================================================
%  GUI — Indicator panel + Start button + Counters
% =====================================================================
fig = figure('Name','IFF Transponder — ZedBoard', ...
    'Position',[100 50 1000 750], 'NumberTitle','off');

% --- Status indicator (color changes for RX / TX / IDLE) ---
ind_panel = uipanel('Parent',fig, 'Title','STATUS', ...
    'Units','normalized', 'Position',[0.01 0.91 0.18 0.08], ...
    'FontSize',10, 'FontWeight','bold');
ind_label = uicontrol('Parent',ind_panel, 'Style','text', ...
    'String','IDLE', 'FontSize',13, 'FontWeight','bold', ...
    'BackgroundColor',[0.6 0.6 0.6], 'ForegroundColor','k', ...
    'Units','normalized', 'Position',[0.02 0.05 0.96 0.90]);

% --- Start button (active only in triggered mode) ---
btn_start = uicontrol('Parent',fig, 'Style','pushbutton', ...
    'String',sprintf('START %d CYCLES', CYCLES_PER_BURST), ...
    'FontSize',11, 'FontWeight','bold', ...
    'Units','normalized', 'Position',[0.21 0.92 0.18 0.06], ...
    'BackgroundColor',[0.3 0.8 0.3], ...
    'Callback',@(~,~) set(fig,'UserData','go'));
if strcmp(OPERATION_MODE, 'continuous')
    set(btn_start, 'Enable','off', 'String','CONTINUOUS MODE');
end

% --- Counter display ---
counter_h = uicontrol('Parent',fig, 'Style','text', ...
    'String','Bursts: 0  |  Interrogations: 0  |  Replies: 0', ...
    'Units','normalized', 'Position',[0.41 0.92 0.57 0.06], ...
    'FontSize',10, 'HorizontalAlignment','center', ...
    'BackgroundColor',get(fig,'Color'));

% =====================================================================
%  CONSOLE BANNER
% =====================================================================
fprintf('========================================================\n');
fprintf('   IFF AIRCRAFT TRANSPONDER — ZedBoard FMCOMMS3\n');
fprintf('========================================================\n');
fprintf('Aircraft ICAO  : %s\n', ICAO_HEX);
fprintf('ZedBoard       : %s\n', ip_zed);
fprintf('Mode           : %s\n', upper(OPERATION_MODE));
fprintf('Cycles/burst   : %d\n', CYCLES_PER_BURST);
fprintf('Listen freq    : %.0f MHz\n', fc_interrogate/1e6);
fprintf('Reply freq     : %.0f MHz\n', fc_reply/1e6);
fprintf('Fs             : %.0f MSPS\n', fs/1e6);
fprintf('RX Gain        : %d dB   TX Gain: %d dB\n', rx_gain, tx_gain);
fprintf('========================================================\n\n');

% =====================================================================
%  MAIN TRANSPONDER LOOP
% =====================================================================
total_bursts     = 0;
n_interrogations = 0;
n_replies        = 0;

set(fig, 'UserData', '');

while ishandle(fig)

    % =================================================================
    %  TRIGGERED MODE: wait for START button press
    % =================================================================
    if strcmp(OPERATION_MODE, 'triggered')
        update_indicator(ind_label, 'IDLE');
        set(btn_start, 'Enable', 'on');
        fprintf('Press START button for %d-cycle burst ...\n', CYCLES_PER_BURST);

        set(fig, 'UserData', '');
        while ishandle(fig) && ~strcmp(get(fig,'UserData'), 'go')
            drawnow;
            pause(0.15);
        end
        if ~ishandle(fig), break; end

        set(btn_start, 'Enable', 'off');
        set(fig, 'UserData', '');
        total_bursts = total_bursts + 1;
        fprintf('\n>>> Burst %d started — %d cycles\n\n', ...
            total_bursts, CYCLES_PER_BURST);
    else
        total_bursts = total_bursts + 1;
    end

    % =================================================================
    %  RUN CYCLES_PER_BURST CYCLES
    % =================================================================
    for cycle = 1:CYCLES_PER_BURST
        if ~ishandle(fig), break; end

        fprintf('=== Cycle %d/%d  [%s] ===\n', ...
            cycle, CYCLES_PER_BURST, datestr(now,'HH:MM:SS'));

        rx_iq        = [];
        interrog_ok  = false;
        interrog_hex = '';
        rx_bits      = [];
        ground_id    = '';
        challenge    = '';

        % -------------------------------------------------------------
        %  STEP 1 — Listen on 1030 MHz   (indicator: BLUE)
        % -------------------------------------------------------------
        update_indicator(ind_label, 'RX');
        drawnow;

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
                mx    = max(abs(rx_iq));
                if mx < 0.02, continue; end

                mag = adsb_preprocess(rx_iq, fs);
                [data_start, ~, snr_ratio] = adsb_framesync(mag, fs);

                if snr_ratio >= 4
                    rx_bits      = adsb_decode_bits(mag, data_start, fs, 88);
                    interrog_hex = bits2hex(rx_bits);

                    if strncmp(interrog_hex, INTERROG_SYNC, 2)
                        interrog_ok      = true;
                        n_interrogations = n_interrogations + 1;
                        ground_id  = interrog_hex(3:6);
                        mode_code  = interrog_hex(7:8);
                        challenge  = interrog_hex(9:16);
                        fprintf('  [RX] INTERROGATION RECEIVED\n');
                        fprintf('       Frame     : %s\n', interrog_hex);
                        fprintf('       Ground ID : %s\n', ground_id);
                        fprintf('       Mode Code : %s\n', mode_code);
                        fprintf('       Challenge : %s\n', challenge);
                        break;
                    else
                        fprintf('  [RX] Frame found but wrong sync: %s\n', ...
                            interrog_hex(1:min(4,end)));
                    end
                end
            end
            release(zrx);

            if ~interrog_ok
                fprintf('  [RX] No valid interrogation in %d s.\n', ...
                    LISTEN_TIMEOUT);
            end
        catch ME
            fprintf('  [RX ERROR] %s\n', ME.message);
            try, release(zrx); catch, end %#ok<NOCOM>
        end

        % -------------------------------------------------------------
        %  STEP 2 — TX reply on 1090 MHz   (indicator: GREEN)
        %
        %  ADS-B Extended Squitter reply (112 bits):
        %  [DF=8D][ICAO 4840D6 (24b)][ME (56b)][PI/CRC (24b)]
        % -------------------------------------------------------------
        if interrog_ok
            update_indicator(ind_label, 'TX');
            drawnow;

            reply_df  = '8D';
            reply_pi  = challenge(1:6);
            reply_hex = [reply_df, ICAO_HEX, SQUAWK_HEX, reply_pi];

            fprintf('  [TX] Replying on %.0f MHz ...\n', fc_reply/1e6);
            fprintf('       Reply frame: %s\n', reply_hex);
            fprintf('       ICAO addr  : %s  (this aircraft)\n', ICAO_HEX);

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

                n_replies = n_replies + 1;
                fprintf('  [TX] Reply sent — aircraft identity transmitted.\n');
            catch ME
                fprintf('  [TX ERROR] %s\n', ME.message);
                try, release(ztx); catch, end %#ok<NOCOM>
            end
        else
            fprintf('  [--] No reply — no valid interrogation.\n');
        end

        % -------------------------------------------------------------
        %  LIVE PLOTS
        % -------------------------------------------------------------
        if ishandle(fig) && ~isempty(rx_iq)
            figure(fig);

            subplot(3,2,1);
            plot(real(rx_iq));
            title(sprintf('Interrogation RX — Cycle %d', cycle));
            xlabel('Sample'); ylabel('I'); grid on;

            subplot(3,2,2);
            mag_plot = adsb_preprocess(rx_iq, fs);
            plot(mag_plot); title('Envelope after DC removal');
            xlabel('Sample'); grid on;

            subplot(3,2,3);
            N = length(rx_iq); f = (-N/2:N/2-1)*(fs/N);
            plot(f/1e6, 20*log10(abs(fftshift(fft(rx_iq)))+eps));
            title('Spectrum'); xlabel('MHz'); ylabel('dB'); grid on;

            subplot(3,2,4);
            if interrog_ok && ~isempty(rx_bits)
                stem(rx_bits(1:min(56,end)),'filled','MarkerSize',3);
                title(sprintf('Interrogation bits (ID: %s)', ground_id));
            else
                cla; title('No interrogation decoded');
            end
            ylim([-0.2 1.2]); grid on;

            subplot(3,2,[5 6]); cla; axis off;
            if interrog_ok
                set(gca,'Color',[0.2 0.8 0.2]);
                text(0.5, 0.5, sprintf( ...
                    'Cycle %d — INTERROGATION RECEIVED\nGround: %s   Challenge: %s\nReply: ICAO %s on %.0f MHz', ...
                    cycle, ground_id, challenge, ICAO_HEX, fc_reply/1e6), ...
                    'FontSize',12, 'FontWeight','bold', ...
                    'HorizontalAlignment','center', ...
                    'VerticalAlignment','middle');
            else
                set(gca,'Color',[0.8 0.2 0.2]);
                text(0.5, 0.5, ...
                    sprintf('Cycle %d — NO INTERROGATION', cycle), ...
                    'FontSize',14, 'FontWeight','bold', ...
                    'HorizontalAlignment','center', ...
                    'VerticalAlignment','middle');
            end
            drawnow;
        end

        % Update counter bar
        if ishandle(fig)
            set(counter_h, 'String', sprintf( ...
                'Bursts: %d  |  Interrogations: %d  |  Replies: %d', ...
                total_bursts, n_interrogations, n_replies));
        end

        pause(0.5);
    end % for cycle

end % while ishandle(fig)

% =====================================================================
%  SESSION SUMMARY
% =====================================================================
fprintf('\n========================================================\n');
fprintf('           TRANSPONDER SESSION SUMMARY\n');
fprintf('========================================================\n');
fprintf('Aircraft ICAO    : %s\n', ICAO_HEX);
fprintf('Operation mode   : %s\n', OPERATION_MODE);
fprintf('Total bursts     : %d\n', total_bursts);
fprintf('Interrogations RX: %d\n', n_interrogations);
fprintf('Replies TX       : %d\n', n_replies);
if n_replies > 0 && n_replies == n_interrogations
    fprintf('ALL interrogations answered — transponder link solid.\n');
elseif n_replies > 0
    fprintf('Some cycles missed — check RX gain or timing.\n');
else
    fprintf('No interrogations received — check ground station.\n');
end
fprintf('========================================================\n');


% #####################################################################
%  LOCAL FUNCTIONS
% #####################################################################

% =====================================================================
%  INDICATOR HELPER
%
%  For IP core: replace with physical LED writes
%    LED0 (blue)  -> RX active
%    LED1 (green) -> TX active
% =====================================================================
function update_indicator(label_h, state)
    switch state
        case 'RX'
            set(label_h, 'String', 'LISTENING', ...
                'BackgroundColor', [0.2 0.5 1.0], ...
                'ForegroundColor', 'w');
        case 'TX'
            set(label_h, 'String', 'TRANSMITTING', ...
                'BackgroundColor', [0.1 0.85 0.2], ...
                'ForegroundColor', 'w');
        case 'IDLE'
            set(label_h, 'String', 'IDLE', ...
                'BackgroundColor', [0.6 0.6 0.6], ...
                'ForegroundColor', 'k');
    end
end

% =====================================================================
%  IP CORE CANDIDATE — Stage 1: Envelope + DC removal
%
%  HDL: movmean -> sliding-window accumulator (shift reg + adder tree)
%       max-norm -> max tracker + single multiplier
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
%  IP CORE CANDIDATE — Stage 2: Preamble matched filter / frame sync
%
%  HDL: xcorr -> FIR with 96-tap ROM coefficients (compile-time constant)
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
%  IP CORE CANDIDATE — Stage 3: PPM bit decisions
%
%  HDL: per-symbol accumulate-and-compare (adder + comparator, no div)
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
