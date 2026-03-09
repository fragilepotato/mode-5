%% MODE 5 IFF — BPSK + CRC-16  (Step 2: No DSSS, No MAC yet)
% Hardware: Pluto SDR (Interrogator) <-SMA-> ZedBoard/FMCOMMS3 (Transponder)
%
% Changes from previous version:
%   - 12 MSPS  (was 4 MSPS — pulse timing was broken)
%   - BPSK modulation  (was PPM)
%   - No DSSS spreading  (removed Gold-31 code)
%   - movmean DC removal  (was highpass — was destroying pulses)
%   - CRC-16 validation only  (MAC layer removed for now)
%   - Polarity-tolerant correlator  (handles LO phase ambiguity)
%
% Frame format:
%   Interrogation : [Sync16][Type8][ID16][Challenge32][CRC16]          = 88 bits
%   Reply         : [Sync16][Type8][ID16][Challenge32][IFF24][CRC16]   = 112 bits
clear all; close all; clc;

% =====================================================================
%  1. CONFIGURATION
% =====================================================================
hw_interrogator = 'Pluto';
hw_transponder  = 'AD936x';
ip_pluto        = 'ip:192.168.2.1';
ip_zed          = '192.168.1.10';

fc_interrogate = 1030e6;
fc_reply       = 1090e6;
fs             = 12e6;          % 12 MSPS (12 samples/µs = 12 samples/bit)

MAX_CYCLES     = 5;
CYCLE_INTERVAL = 5;             % seconds between cycles

% SMA cable gains — target max(abs(rx)) ~ 0.2–0.6
tx_gain_pluto = -20;
tx_gain_zed   = -20;
rx_gain_zed   = 10;
rx_gain_pluto = 10;

% Mode 5 identity
interrogator_id = uint16(hex2dec('1A2B'));
transponder_id  = uint16(hex2dec('3C4D'));
iff_code        = uint32(hex2dec('00F01E'));

% BPSK preamble (16 chips, BPSK symbols ±1) — sent before data for frame sync
% Chosen for good autocorrelation without side-lobes
PREAMBLE_SYMS = [1 -1 1 -1 1 -1 1 -1 1 1 -1 -1 1 1 -1 -1];

% =====================================================================
%  2. BANNER
% =====================================================================
disp('========================================================');
disp('   MODE 5 IFF — BPSK + CRC-16  (Step 2)');
disp('========================================================');
fprintf('Interrogator : %s  (%s)\n', hw_interrogator, ip_pluto);
fprintf('Transponder  : %s  (%s)\n', hw_transponder,  ip_zed);
fprintf('Interrogation: %.0f MHz\n', fc_interrogate/1e6);
fprintf('Reply        : %.0f MHz\n', fc_reply/1e6);
fprintf('Sample Rate  : %.0f MSPS  (%d samp/bit)\n', fs/1e6, round(fs/1e6));
fprintf('Modulation   : BPSK  |  Validation: CRC-16 only\n');
fprintf('Cycles       : %d  |  Interval: %d s\n', MAX_CYCLES, CYCLE_INTERVAL);
disp('========================================================');
disp('Press Ctrl-C or close the figure to stop.');
disp(' ');

% =====================================================================
%  3. MAIN IFF LOOP
% =====================================================================
cycle_count = 0;
fig = figure('Name','Mode 5 IFF Monitor','Position',[100 100 950 700]);

while ishandle(fig) && cycle_count < MAX_CYCLES
    cycle_count = cycle_count + 1;
    fprintf('\n=== IFF Cycle #%d/%d  [%s] ===\n', ...
        cycle_count, MAX_CYCLES, datestr(now,'HH:MM:SS'));

    challenge = randi([0, 2^32-1], 1, 'uint32');
    fprintf('  Challenge : 0x%08X\n', challenge);

    rx_data_interrog = [];
    rx_data_reply    = [];
    snr_i = 0;  snr_r = 0;
    iff_status = 'NO REPLY';

    % =================================================================
    %  PHASE 1 — INTERROGATION  (Pluto TX -> ZedBoard RX)
    % =================================================================
    fprintf('  [PHASE 1] Interrogation on %.0f MHz ...\n', fc_interrogate/1e6);

    interrog_bits = build_interrogation(interrogator_id, challenge);
    tx_interrog   = bpsk_modulate(interrog_bits, PREAMBLE_SYMS, fs);

    try
        ptx = sdrtx(hw_interrogator, 'RadioID', ip_pluto, ...
            'CenterFrequency', fc_interrogate, 'BasebandSampleRate', fs);
        ptx.Gain = tx_gain_pluto;

        zrx = sdrrx(hw_transponder, 'IPAddress', ip_zed, ...
            'CenterFrequency', fc_interrogate, 'BasebandSampleRate', fs, ...
            'SamplesPerFrame', length(tx_interrog)*4, ...
            'OutputDataType', 'double', 'ChannelMapping', 1);
        zrx.GainSource = 'Manual';
        zrx.Gain = rx_gain_zed;

        transmitRepeat(ptx, tx_interrog);
        pause(1.0);
        for k = 1:3, zrx(); end      % flush stale buffer
        rx_data_interrog = zrx();
        release(ptx); release(zrx);

        rx_mag = abs(rx_data_interrog);
        fprintf('  [DEBUG] RX: samples=%d  max=%.4f  mean=%.4f\n', ...
            length(rx_data_interrog), max(rx_mag), mean(rx_mag));

        [rx_bits_i, snr_i, dbg_i] = bpsk_demodulate( ...
            rx_data_interrog, PREAMBLE_SYMS, fs, length(interrog_bits));

        fprintf('  [DEBUG] Demod: SNR=%.1fdB  start=%d  bits=%d/%d  phase=%.1fdeg\n', ...
            snr_i, dbg_i.data_start, length(rx_bits_i), ...
            length(interrog_bits), dbg_i.phase_deg);

        if length(rx_bits_i) == length(interrog_bits)
            fprintf('  [DEBUG] Bit errors: %d/%d\n', ...
                sum(rx_bits_i ~= interrog_bits), length(interrog_bits));
        end

        [rx_iid, rx_chal, interrog_ok] = parse_interrogation(rx_bits_i);

        if interrog_ok
            fprintf('  [TRANSPONDER] OK — ID=0x%04X  Challenge=0x%08X  SNR=%.1fdB\n', ...
                rx_iid, rx_chal, snr_i);
        else
            fprintf('  [TRANSPONDER] FAILED — interrogation rejected.\n');
            wait_and_continue(fig, CYCLE_INTERVAL);
            continue;
        end
    catch ME
        fprintf('  [ERROR] Phase 1: %s\n', ME.message);
        try, release(ptx); catch, end
        try, release(zrx); catch, end
        wait_and_continue(fig, CYCLE_INTERVAL);
        continue;
    end

    % =================================================================
    %  PHASE 2 — REPLY  (ZedBoard TX -> Pluto RX)
    % =================================================================
    fprintf('  [PHASE 2] Reply on %.0f MHz ...\n', fc_reply/1e6);

    reply_bits = build_reply(transponder_id, iff_code, rx_chal);
    tx_reply   = bpsk_modulate(reply_bits, PREAMBLE_SYMS, fs);

    try
        ztx = sdrtx(hw_transponder, 'IPAddress', ip_zed, ...
            'CenterFrequency', fc_reply, 'BasebandSampleRate', fs, ...
            'ChannelMapping', 1);
        ztx.Gain = tx_gain_zed;

        prx = sdrrx(hw_interrogator, 'RadioID', ip_pluto, ...
            'CenterFrequency', fc_reply, 'BasebandSampleRate', fs, ...
            'SamplesPerFrame', length(tx_reply)*4, ...
            'OutputDataType', 'double');
        prx.GainSource = 'Manual';
        prx.Gain = rx_gain_pluto;

        transmitRepeat(ztx, tx_reply);
        pause(1.0);
        for k = 1:3, prx(); end
        rx_data_reply = prx();
        release(ztx); release(prx);

        rx_mag_r = abs(rx_data_reply);
        fprintf('  [DEBUG] Reply RX: max=%.4f  mean=%.4f\n', ...
            max(rx_mag_r), mean(rx_mag_r));

        [rx_bits_r, snr_r, dbg_r] = bpsk_demodulate( ...
            rx_data_reply, PREAMBLE_SYMS, fs, length(reply_bits));

        fprintf('  [DEBUG] Reply demod: SNR=%.1fdB  start=%d  bits=%d/%d  phase=%.1fdeg\n', ...
            snr_r, dbg_r.data_start, length(rx_bits_r), length(reply_bits), dbg_r.phase_deg);

        if length(rx_bits_r) == length(reply_bits)
            fprintf('  [DEBUG] Reply bit errors: %d/%d\n', ...
                sum(rx_bits_r ~= reply_bits), length(reply_bits));
        end

        [rx_tid, rx_ifc, reply_ok] = parse_reply(rx_bits_r, challenge);

        if reply_ok
            fprintf('  [INTERROGATOR] OK — Transponder=0x%04X  IFF=0x%06X  SNR=%.1fdB\n', ...
                rx_tid, rx_ifc, snr_r);
            fprintf('  *** RESULT: FRIEND IDENTIFIED ***\n');
            iff_status = 'FRIEND';
        else
            fprintf('  [INTERROGATOR] FAILED — reply rejected.\n');
            fprintf('  *** RESULT: UNKNOWN ***\n');
            iff_status = 'UNKNOWN';
        end
    catch ME
        fprintf('  [ERROR] Phase 2: %s\n', ME.message);
        try, release(ztx); catch, end
        try, release(prx); catch, end
        iff_status = 'NO REPLY';
    end

    update_monitor(fig, cycle_count, iff_status, ...
        rx_data_interrog, rx_data_reply, snr_i, snr_r, fs);

    if cycle_count < MAX_CYCLES
        fprintf('  Next cycle in %d s ...\n', CYCLE_INTERVAL);
        pause(CYCLE_INTERVAL);
    end
end

disp('Mode 5 IFF system stopped.');

% =====================================================================
%  BPSK MODULATOR
% =====================================================================
function tx = bpsk_modulate(bits, preamble_syms, fs)
    sps   = round(fs / 1e6);                  % 12 samples per bit at 12 MSPS
    guard = zeros(1, sps * 20);               % 20 µs guard

    % Preamble: each ±1 chip repeated sps times
    pre_sig = repelem(preamble_syms, sps);

    % Data: BPSK symbols (0->-1, 1->+1), each repeated sps times
    bpsk_syms = 2*double(bits) - 1;
    data_sig  = repelem(bpsk_syms, sps);

    sig = double([guard, pre_sig, data_sig, guard]);
    sig = sig / max(abs(sig)) * 0.9;

    % Force complex I/Q for Pluto (1e-12 imaginary prevents MATLAB
    % from collapsing type back to real)
    tx = sig.' + 1i * 1e-12 * ones(length(sig), 1);
end

% =====================================================================
%  BPSK DEMODULATOR
% =====================================================================
function [bits, snr_est, dbg] = bpsk_demodulate(rx_sig, preamble_syms, fs, n_bits_expected)
    sps      = round(fs / 1e6);
    rx_sig   = rx_sig(:).';           % row vector, keep complex

    % Step 1: Build preamble template (real ±1, chip-oversampled)
    pre_tmpl = repelem(preamble_syms, sps);

    % Step 2: Correlate COMPLEX rx against real preamble.
    %   xcorr(complex, real) → complex result.
    %   abs() is phase-invariant → reliable peak regardless of carrier phase.
    %   angle() at peak = carrier phase offset θ between TX and RX LOs.
    [c, lags] = xcorr(rx_sig, pre_tmpl);
    [peak_val, pk_idx] = max(abs(c));
    noise_est = median(abs(c));
    snr_est   = 20 * log10(peak_val / (noise_est + eps));

    % Step 3: Rotate signal by -θ to align BPSK constellation to real axis.
    %   After rotation:  +1 symbol → real component positive
    %                    -1 symbol → real component negative
    phase_offset = angle(c(pk_idx));
    rx_rot = rx_sig .* exp(-1i * phase_offset);
    rx_r   = real(rx_rot);

    % Step 4: Remove residual DC (AD9361 baseband ramp, 20 µs window)
    dc_win = round(20 * sps);
    rx_r   = rx_r - movmean(rx_r, dc_win);

    % Step 5: Data start = preamble start + preamble length
    start_idx  = lags(pk_idx) + 1;
    data_start = start_idx + length(pre_tmpl);
    if data_start < 1, data_start = 1; end

    % Step 6: Decide each bit by integrating over one symbol period
    n_bits = min(n_bits_expected, floor((length(rx_r) - data_start) / sps));
    bits   = zeros(1, n_bits);
    for k = 0:n_bits-1
        i0 = data_start + k*sps;
        i1 = i0 + sps - 1;
        if i1 > length(rx_r), break; end
        bits(k+1) = double(sum(rx_r(i0:i1)) > 0);
    end

    dbg.data_start   = data_start;
    dbg.phase_deg    = phase_offset * 180 / pi;
    dbg.peak_val     = peak_val;
    dbg.noise_est    = noise_est;
end

% =====================================================================
%  FRAME BUILDERS  (CRC-16 only — no MAC)
% =====================================================================
function bits = build_interrogation(interrog_id, challenge)
    % [Sync16][Type8][ID16][Challenge32][CRC16] = 88 bits
    sync  = [1 0 1 0 1 0 1 0 1 1 0 0 1 1 0 0];
    mtype = [0 0 0 0 0 0 0 1];
    id_b  = dec2binvec(interrog_id, 16);
    ch_b  = dec2binvec(challenge,   32);
    pre   = [sync, mtype, id_b, ch_b];
    bits  = [pre, dec2binvec(compute_crc16(pre), 16)];
end

function bits = build_reply(transp_id, iff_code, challenge)
    % Echo challenge in reply so interrogator can validate it
    % [Sync16][Type8][ID16][Challenge32][IFF24][CRC16] = 112 bits
    sync  = [1 0 1 0 1 0 1 0 1 1 0 0 1 1 0 0];
    mtype = [0 0 0 0 0 0 1 0];
    id_b  = dec2binvec(transp_id,        16);
    ch_b  = dec2binvec(challenge,        32);
    iff_b = dec2binvec(uint32(iff_code), 24);
    pre   = [sync, mtype, id_b, ch_b, iff_b];
    bits  = [pre, dec2binvec(compute_crc16(pre), 16)];
end

% =====================================================================
%  FRAME PARSERS
% =====================================================================
function [iid, chal, valid] = parse_interrogation(bits)
    valid = false; iid = 0; chal = uint32(0);
    if length(bits) < 88
        fprintf('  [DEBUG PARSE] Too few bits: %d (need 88)\n', length(bits));
        return;
    end
    sync_rx = bits(1:16);
    sync_ok = isequal(sync_rx, [1 0 1 0 1 0 1 0 1 1 0 0 1 1 0 0]);
    if ~sync_ok
        fprintf('  [DEBUG PARSE] Sync FAIL: got [%s]\n', num2str(sync_rx));
        return;
    end
    crc_calc = compute_crc16(bits(1:72));
    crc_recv = binvec2dec(bits(73:88));
    if crc_calc ~= crc_recv
        fprintf('  [DEBUG PARSE] CRC FAIL: calc=0x%04X  recv=0x%04X\n', ...
            crc_calc, crc_recv);
        return;
    end
    iid   = binvec2dec(bits(25:40));
    chal  = uint32(binvec2dec(bits(41:72)));
    valid = true;
end

function [tid, ifc, valid] = parse_reply(bits, expected_challenge)
    valid = false; tid = 0; ifc = 0;
    if length(bits) < 112
        fprintf('  [DEBUG PARSE] Too few bits: %d (need 112)\n', length(bits));
        return;
    end
    sync_rx = bits(1:16);
    if ~isequal(sync_rx, [1 0 1 0 1 0 1 0 1 1 0 0 1 1 0 0])
        fprintf('  [DEBUG PARSE] Reply sync FAIL: got [%s]\n', num2str(sync_rx));
        return;
    end
    crc_calc = compute_crc16(bits(1:96));
    crc_recv = binvec2dec(bits(97:112));
    if crc_calc ~= crc_recv
        fprintf('  [DEBUG PARSE] Reply CRC FAIL: calc=0x%04X  recv=0x%04X\n', ...
            crc_calc, crc_recv);
        return;
    end
    % Verify echoed challenge matches what we sent
    chal_echo = uint32(binvec2dec(bits(41:72)));
    if chal_echo ~= expected_challenge
        fprintf('  [DEBUG PARSE] Challenge mismatch: got=0x%08X  expect=0x%08X\n', ...
            chal_echo, expected_challenge);
        return;
    end
    tid   = binvec2dec(bits(25:40));
    ifc   = binvec2dec(bits(73:96));
    valid = true;
end

% =====================================================================
%  UTILITIES
% =====================================================================
function bv = dec2binvec(val, nbits)
    v  = uint64(val);
    bv = zeros(1, nbits);
    for i = 1:nbits
        bv(i) = double(bitand(bitshift(v, -(nbits-i)), uint64(1)));
    end
end

function val = binvec2dec(bv)
    val = uint32(0);
    for i = 1:length(bv)
        val = bitor(bitshift(val, 1), uint32(bv(i)));
    end
end

function crc = compute_crc16(bits)
    poly = uint16(hex2dec('1021'));
    crc  = uint16(hex2dec('FFFF'));
    for b = bits
        hi  = bitshift(crc, -15);
        crc = bitshift(crc, 1);
        if bitxor(hi, uint16(b))
            crc = bitxor(crc, poly);
        end
    end
end

% =====================================================================
%  LIVE MONITOR
% =====================================================================
function update_monitor(fig, cycle, status, rx_i, rx_r, snr_i, snr_r, fs)
    if ~ishandle(fig), return; end
    figure(fig);

    subplot(3,2,1);
    if ~isempty(rx_i), plot(real(rx_i)); end
    title(sprintf('Interrogation RX — Cycle %d', cycle));
    xlabel('Sample'); ylabel('I'); grid on;

    subplot(3,2,2);
    if ~isempty(rx_r), plot(real(rx_r)); end
    title(sprintf('Reply RX — Cycle %d', cycle));
    xlabel('Sample'); ylabel('I'); grid on;

    subplot(3,2,3);
    if ~isempty(rx_i)
        N = length(rx_i);
        f = (-N/2:N/2-1)*(fs/N);
        plot(f/1e6, 20*log10(abs(fftshift(fft(rx_i)))+eps));
        xlabel('MHz'); ylabel('dB'); grid on;
    end
    title('Interrogation Spectrum');

    subplot(3,2,4);
    if ~isempty(rx_r)
        N = length(rx_r);
        f = (-N/2:N/2-1)*(fs/N);
        plot(f/1e6, 20*log10(abs(fftshift(fft(rx_r)))+eps));
        xlabel('MHz'); ylabel('dB'); grid on;
    end
    title('Reply Spectrum');

    subplot(3,2,[5 6]); cla; axis off;
    col = struct('FRIEND',[0.2 0.8 0.2],'UNKNOWN',[0.9 0.6 0.1],'NO_REPLY',[0.8 0.2 0.2]);
    if isfield(col, strrep(status,' ','_'))
        bg = col.(strrep(status,' ','_'));
    else
        bg = [0.8 0.2 0.2];
    end
    set(gca, 'Color', bg);
    text(0.5, 0.5, sprintf('Cycle %d  —  %s\nInterrog SNR: %.1f dB    Reply SNR: %.1f dB', ...
        cycle, status, snr_i, snr_r), ...
        'FontSize', 14, 'FontWeight', 'bold', ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
    drawnow;
end

% =====================================================================
%  HELPERS
% =====================================================================
function wait_and_continue(fig, interval)
    fprintf('  Waiting %d s ...\n', interval);
    if ishandle(fig), pause(interval); end
end
