%% MODE 5 IFF COMMUNICATION SYSTEM (Continuous Challenge-Response)
% Hardware: Pluto SDR (Interrogator) <-SMA-> ZedBoard/FMCOMMS3 (Transponder)
%
% Protocol (each cycle):
%   1. Interrogator generates random challenge nonce
%   2. Pluto TX -> ZedBoard RX  (interrogation on 1030 MHz)
%   3. Transponder authenticates and computes crypto response
%   4. ZedBoard TX -> Pluto RX  (reply on 1090 MHz)
%   5. Interrogator validates crypto response
%   6. Result: FRIEND / FOE / NO REPLY
%   7. Wait 5 seconds, repeat
%
% Modulation: BPSK with DSSS (Direct Sequence Spread Spectrum)
% Authentication: HMAC-style MAC with pre-shared 16-byte key
clear all; close all; clc;

% =====================================================================
%  1. CONFIGURATION
% =====================================================================
hw_interrogator = 'Pluto';          % Pluto SDR = Interrogator
hw_transponder  = 'AD936x';         % ZedBoard FMCOMMS3 = Transponder

ip_pluto = 'ip:192.168.2.1';        % Pluto USB IP
ip_zed   = '192.168.1.10';          % ZedBoard Ethernet IP

fc_interrogate = 1030e6;             % Interrogation frequency
fc_reply       = 1090e6;             % Reply frequency
fs             = 4e6;                % 4 MSPS baseband sample rate

CYCLE_INTERVAL = 5;                  % Seconds between IFF cycles

% Gain settings for DIRECT SMA CABLE connection
% SMA cable has ~0 dB path loss — use minimum TX and low RX to avoid clipping
tx_gain_pluto = -50;                 % Pluto TX gain (dB) — very low for cable
tx_gain_zed   = -20;                 % ZedBoard TX gain (dB)
rx_gain_zed   = 0;                   % ZedBoard RX gain (dB)
rx_gain_pluto = 0;                   % Pluto RX gain (dB)

% Mode 5 identity parameters
interrogator_id = uint16(hex2dec('1A2B'));
transponder_id  = uint16(hex2dec('3C4D'));
iff_code        = uint32(hex2dec('00F01E'));   % Friend platform code

% Pre-shared 16-byte key (real Mode 5 uses TSEC/KIV-77 keys)
crypto_key = uint8([73 70 70 95 77 79 68 69 53 95 75 69 89 33 48 49]);

% DSSS spreading: length-31 Gold code -> ~15 dB processing gain
pn_code = [1 -1 1 1 -1 -1 1 -1 1 -1 -1 1 1 1 -1 ...
           1 1 -1 1 1 1 -1 -1 -1 1 -1 -1 1 -1 1 1];
chips_per_bit = length(pn_code);

% =====================================================================
%  2. BANNER
% =====================================================================
disp('========================================================');
disp('        MODE 5 IFF COMMUNICATION SYSTEM');
disp('     Continuous Challenge-Response Protocol');
disp('========================================================');
fprintf('Interrogator : %s  (%s)\n', hw_interrogator, ip_pluto);
fprintf('Transponder  : %s  (%s)\n', hw_transponder,  ip_zed);
fprintf('Interrogation: %.0f MHz\n', fc_interrogate/1e6);
fprintf('Reply        : %.0f MHz\n', fc_reply/1e6);
fprintf('Sample Rate  : %.0f MSPS\n', fs/1e6);
fprintf('Spreading    : %d chips  (%.1f dB gain)\n', ...
    chips_per_bit, 10*log10(chips_per_bit));
fprintf('Cycle Interval: %d s\n', CYCLE_INTERVAL);
disp('========================================================');
disp('Press Ctrl-C or close the figure to stop.');
disp(' ');

% =====================================================================
%  3. MAIN IFF LOOP
% =====================================================================
MAX_CYCLES  = 5;
cycle_count = 0;
fig = figure('Name','Mode 5 IFF Monitor','Position',[100 100 950 700]);

while ishandle(fig) && cycle_count < MAX_CYCLES
    cycle_count = cycle_count + 1;
    t_str = datestr(now, 'HH:MM:SS');
    fprintf('\n=== IFF Cycle #%d  [%s] ===\n', cycle_count, t_str);

    % Fresh challenge nonce each cycle
    challenge = randi([0, 2^32-1], 1, 'uint32');
    fprintf('  Challenge nonce : 0x%08X\n', challenge);

    rx_data_interrog = [];
    rx_data_reply    = [];
    snr_i = 0;  snr_r = 0;
    iff_status = 'NO REPLY';

    % =================================================================
    %  PHASE 1 — INTERROGATION  (Pluto TX -> ZedBoard RX)  1030 MHz
    % =================================================================
    fprintf('  [PHASE 1] TX interrogation on %.0f MHz ...\n', ...
        fc_interrogate/1e6);

    interrog_bits = build_interrogation(interrogator_id, challenge, crypto_key);
    tx_interrog   = dsss_modulate(interrog_bits, pn_code, fs);

    try
        ptx = sdrtx(hw_interrogator, 'RadioID', ip_pluto, ...
            'CenterFrequency', fc_interrogate, 'BasebandSampleRate', fs);
        ptx.Gain = tx_gain_pluto;

        zrx = sdrrx(hw_transponder, 'IPAddress', ip_zed, ...
            'CenterFrequency', fc_interrogate, 'BasebandSampleRate', fs, ...
            'SamplesPerFrame', length(tx_interrog)*3, ...
            'OutputDataType', 'double', 'ChannelMapping', 1);
        zrx.GainSource = 'Manual';
        zrx.Gain = rx_gain_zed;

        transmitRepeat(ptx, tx_interrog);
        pause(0.5);                % Extra settle time for SMA cable
        for flush = 1:3, zrx(); end % Flush stale RX buffer
        rx_data_interrog = zrx();
        release(ptx); release(zrx);

        % --- DEBUG: RX signal stats ---
        rx_mag = abs(rx_data_interrog);
        fprintf('  [DEBUG] RX samples=%d  max=%.4f  mean=%.4f  std=%.4f\n', ...
            length(rx_data_interrog), max(rx_mag), mean(rx_mag), std(rx_mag));

        [rx_bits_i, snr_i, dbg_i] = dsss_demodulate(rx_data_interrog, pn_code, fs, length(interrog_bits));
        fprintf('  [DEBUG] Demod: SNR=%.1f dB  data_start=%d  bits_extracted=%d  expected=%d\n', ...
            snr_i, dbg_i.data_start, length(rx_bits_i), length(interrog_bits));
        fprintf('  [DEBUG] TX bits (first 20): %s\n', num2str(interrog_bits(1:min(20,end))));
        fprintf('  [DEBUG] RX bits (first 20): %s\n', num2str(rx_bits_i(1:min(20,end))));
        if length(rx_bits_i) == length(interrog_bits)
            bit_errs = sum(rx_bits_i ~= interrog_bits);
            fprintf('  [DEBUG] Bit errors: %d / %d  (BER=%.2f%%)\n', ...
                bit_errs, length(interrog_bits), 100*bit_errs/length(interrog_bits));
        end

        [rx_iid, rx_chal, interrog_ok] = parse_interrogation(rx_bits_i, crypto_key);

        if interrog_ok
            fprintf('  [TRANSPONDER] Valid interrogation. ID=0x%04X  Challenge=0x%08X  SNR=%.1f dB\n', ...
                rx_iid, rx_chal, snr_i);
        else
            fprintf('  [TRANSPONDER] Interrogation FAILED validation.\n');
            wait_and_continue(fig, CYCLE_INTERVAL);
            continue;
        end
    catch ME
        fprintf('  [ERROR] Phase 1: %s\n', ME.message);
        safe_release('ptx','zrx');
        wait_and_continue(fig, CYCLE_INTERVAL);
        continue;
    end

    % =================================================================
    %  PHASE 2 — REPLY  (ZedBoard TX -> Pluto RX)  1090 MHz
    % =================================================================
    fprintf('  [PHASE 2] TX reply on %.0f MHz ...\n', fc_reply/1e6);

    reply_bits = build_reply(transponder_id, iff_code, rx_chal, crypto_key);
    tx_reply   = dsss_modulate(reply_bits, pn_code, fs);

    try
        ztx = sdrtx(hw_transponder, 'IPAddress', ip_zed, ...
            'CenterFrequency', fc_reply, 'BasebandSampleRate', fs, ...
            'ChannelMapping', 1);
        ztx.Gain = tx_gain_zed;

        prx = sdrrx(hw_interrogator, 'RadioID', ip_pluto, ...
            'CenterFrequency', fc_reply, 'BasebandSampleRate', fs, ...
            'SamplesPerFrame', length(tx_reply)*3, ...
            'OutputDataType', 'double');
        prx.GainSource = 'Manual';
        prx.Gain = rx_gain_pluto;

        transmitRepeat(ztx, tx_reply);
        pause(0.5);
        for flush = 1:3, prx(); end % Flush stale RX buffer
        rx_data_reply = prx();
        release(ztx); release(prx);

        % --- DEBUG: Reply RX signal stats ---
        rx_mag_r = abs(rx_data_reply);
        fprintf('  [DEBUG] Reply RX samples=%d  max=%.4f  mean=%.4f\n', ...
            length(rx_data_reply), max(rx_mag_r), mean(rx_mag_r));

        [rx_bits_r, snr_r, dbg_r] = dsss_demodulate(rx_data_reply, pn_code, fs, length(reply_bits));
        fprintf('  [DEBUG] Reply demod: SNR=%.1f dB  data_start=%d  bits=%d  expected=%d\n', ...
            snr_r, dbg_r.data_start, length(rx_bits_r), length(reply_bits));
        if length(rx_bits_r) == length(reply_bits)
            fprintf('  [DEBUG] Reply bit errors: %d / %d\n', ...
                sum(rx_bits_r ~= reply_bits), length(reply_bits));
        end
        [rx_tid, rx_ifc, ~, reply_ok] = ...
            parse_reply(rx_bits_r, challenge, crypto_key);

        if reply_ok
            fprintf('  [INTERROGATOR] Valid reply. Transponder=0x%04X  IFF=0x%06X  SNR=%.1f dB\n', ...
                rx_tid, rx_ifc, snr_r);
            fprintf('  *** RESULT: FRIEND IDENTIFIED ***\n');
            iff_status = 'FRIEND';
        else
            fprintf('  [INTERROGATOR] Reply FAILED crypto validation.\n');
            fprintf('  *** RESULT: UNKNOWN / POTENTIAL FOE ***\n');
            iff_status = 'UNKNOWN';
        end
    catch ME
        fprintf('  [ERROR] Phase 2: %s\n', ME.message);
        safe_release('ztx','prx');
        iff_status = 'NO REPLY';
    end

    % =================================================================
    %  UPDATE LIVE MONITOR
    % =================================================================
    update_monitor(fig, cycle_count, iff_status, ...
        rx_data_interrog, rx_data_reply, snr_i, snr_r, fs);

    % =================================================================
    %  WAIT FOR NEXT CYCLE
    % =================================================================
    fprintf('  Next cycle in %d s ...\n', CYCLE_INTERVAL);
    pause(CYCLE_INTERVAL);
end

disp('Mode 5 IFF system stopped.');

% =====================================================================
%  MESSAGE CONSTRUCTION
% =====================================================================
function bits = build_interrogation(interrog_id, challenge, key)
    % [Sync 16][Type 8][InterrogID 16][Challenge 32][MAC 32][CRC 16] = 120 b
    sync  = [1 0 1 0 1 0 1 0 1 1 0 0 1 1 0 0];   % 0xAACC
    mtype = [0 0 0 0 0 0 0 1];                     % 0x01
    id_b  = dec2binvec(interrog_id, 16);
    ch_b  = dec2binvec(challenge, 32);

    payload  = [mtype, id_b, ch_b];
    mac_bits = dec2binvec(compute_mac(payload, key), 32);

    pre_crc = [sync, mtype, id_b, ch_b, mac_bits];
    crc_b   = dec2binvec(compute_crc16(pre_crc), 16);
    bits    = [pre_crc, crc_b];
end

function bits = build_reply(transp_id, iff_code, challenge, key)
    % [Sync 16][Type 8][TranspID 16][IFF 24][CryptoResp 32][CRC 16] = 112 b
    sync  = [1 0 1 0 1 0 1 0 1 1 0 0 1 1 0 0];
    mtype = [0 0 0 0 0 0 1 0];                     % 0x02
    id_b  = dec2binvec(transp_id, 16);
    iff_b = dec2binvec(uint32(iff_code), 24);

    ch_b  = dec2binvec(challenge, 32);
    resp  = compute_mac([ch_b, id_b, iff_b], key);
    rsp_b = dec2binvec(resp, 32);

    pre_crc = [sync, mtype, id_b, iff_b, rsp_b];
    crc_b   = dec2binvec(compute_crc16(pre_crc), 16);
    bits    = [pre_crc, crc_b];
end

% =====================================================================
%  MESSAGE PARSING
% =====================================================================
function [iid, chal, valid] = parse_interrogation(bits, key)
    valid = false; iid = 0; chal = uint32(0);
    if length(bits) < 120
        fprintf('  [DEBUG PARSE] Too few bits: %d (need 120)\n', length(bits));
        return;
    end

    sync  = bits(1:16);
    mtype = bits(17:24);
    id_b  = bits(25:40);
    ch_b  = bits(41:72);
    mac_b = bits(73:104);
    crc_b = bits(105:120);

    expected_sync = [1 0 1 0 1 0 1 0 1 1 0 0 1 1 0 0];
    if ~isequal(sync, expected_sync)
        fprintf('  [DEBUG PARSE] Sync FAIL: got [%s]  expect [%s]\n', ...
            num2str(sync), num2str(expected_sync));
        return;
    end

    crc_calc = compute_crc16(bits(1:104));
    crc_recv = binvec2dec(crc_b);
    if crc_calc ~= crc_recv
        fprintf('  [DEBUG PARSE] CRC FAIL: calc=0x%04X  recv=0x%04X\n', crc_calc, crc_recv);
        return;
    end

    mac_calc = compute_mac([mtype, id_b, ch_b], key);
    mac_recv = binvec2dec(mac_b);
    if mac_calc ~= mac_recv
        fprintf('  [DEBUG PARSE] MAC FAIL: calc=0x%08X  recv=0x%08X\n', mac_calc, mac_recv);
        return;
    end

    iid  = binvec2dec(id_b);
    chal = uint32(binvec2dec(ch_b));
    valid = true;
end

function [tid, ifc, cresp, valid] = parse_reply(bits, challenge, key)
    valid = false; tid = 0; ifc = 0; cresp = 0;
    if length(bits) < 112, return; end

    sync  = bits(1:16);
    id_b  = bits(25:40);
    iff_b = bits(41:64);
    rsp_b = bits(65:96);
    crc_b = bits(97:112);

    if ~isequal(sync, [1 0 1 0 1 0 1 0 1 1 0 0 1 1 0 0]), return; end
    if compute_crc16(bits(1:96)) ~= binvec2dec(crc_b), return; end

    tid   = binvec2dec(id_b);
    ifc   = binvec2dec(iff_b);
    cresp = binvec2dec(rsp_b);

    ch_b  = dec2binvec(challenge, 32);
    expected = compute_mac([ch_b, id_b, iff_b], key);
    valid = (cresp == expected);
end

% =====================================================================
%  DSSS MODULATION / DEMODULATION
% =====================================================================
function tx = dsss_modulate(bits, pn, fs)
    chips_per_bit   = length(pn);
    samp_per_chip   = round(fs / 1e6);          % 1 chip ≈ 1 µs

    bpsk   = 2*bits - 1;                         % 0->-1, 1->+1
    spread = zeros(1, length(bpsk)*chips_per_bit);
    for k = 1:length(bpsk)
        spread((k-1)*chips_per_bit+1 : k*chips_per_bit) = bpsk(k) * pn;
    end

    baseband = repelem(spread, samp_per_chip);

    % Preamble: two PN periods (pos, neg) for frame sync
    pre_chips = [pn, -pn];
    pre_samp  = repelem(pre_chips, samp_per_chip);

    guard = zeros(1, samp_per_chip * chips_per_bit);
    sig = double([guard, pre_samp, baseband, guard].');
    sig = sig / max(abs(sig)) * 0.8;              % Normalize
    tx  = sig + 1i*1e-12*ones(size(sig));          % Force complex I/Q for Pluto
end

function [bits, snr_est, dbg] = dsss_demodulate(rx_sig, pn, fs, expected_bits)
    if nargin < 4, expected_bits = 0; end
    chips_per_bit = length(pn);
    samp_per_chip = round(fs / 1e6);

    rx = real(rx_sig(:).');
    rx = rx - mean(rx);                          % Remove DC

    % Highpass filter to remove any DC ramp from AD9361
    try
        rx = highpass(rx, 50e3, fs);
    catch
        rx = [0, diff(rx)];
    end

    pn_samp  = repelem(pn, samp_per_chip);
    pre_tmpl = repelem([pn, -pn], samp_per_chip);

    [corr_pre, lags] = xcorr(rx, pre_tmpl);
    [peak_val, pk] = max(abs(corr_pre));
    data_start = lags(pk) + length(pre_tmpl) + 1;

    noise_floor = median(abs(corr_pre));
    snr_est = 20*log10(peak_val / (noise_floor + eps));

    samp_per_bit = chips_per_bit * samp_per_chip;

    % Determine how many bits to extract
    max_possible = floor((length(rx) - max(data_start,1)) / samp_per_bit);
    if expected_bits > 0
        n_bits = min(expected_bits, max(max_possible, 0));
    else
        n_bits = max(max_possible, 0);
    end

    % Protect against bad sync (data_start < 1)
    if data_start < 1
        data_start = 1;
    end

    bits = zeros(1, n_bits);
    for k = 0:n_bits-1
        i0 = data_start + k*samp_per_bit;
        i1 = i0 + samp_per_bit - 1;
        if i1 > length(rx), break; end
        bits(k+1) = double(sum(rx(i0:i1) .* pn_samp) > 0);
    end

    dbg.data_start  = data_start;
    dbg.peak_val    = peak_val;
    dbg.noise_floor = noise_floor;
    dbg.n_bits      = n_bits;
    dbg.samp_per_bit = samp_per_bit;
end

% =====================================================================
%  CRYPTO HELPERS
% =====================================================================
function mac = compute_mac(bits, key)
    % HMAC-like: H(key XOR opad || H(key XOR ipad || msg))
    n = ceil(length(bits)/8);
    pad_bits = [bits, zeros(1, n*8 - length(bits))];
    data = zeros(1, n, 'uint8');
    for i = 1:n
        data(i) = uint8(pad_bits((i-1)*8+1:i*8) * [128;64;32;16;8;4;2;1]);
    end

    kp = zeros(1, 16, 'uint8');
    kp(1:min(16,length(key))) = key(1:min(16,length(key)));
    ipad = bitxor(kp, uint8(hex2dec('36')*ones(1,16)));
    opad = bitxor(kp, uint8(hex2dec('5C')*ones(1,16)));

    mac = simple_hash([opad, simple_hash_bytes([ipad, data])]);
end

function h = simple_hash(data)
    % 32-bit hash (demo only — real Mode 5 uses AES-128)
    st = uint32(hex2dec('DEADBEEF'));
    for i = 1:length(data)
        st = bitxor(st, bitshift(uint32(data(i)), mod(i*7, 24)));
        st = bitxor(st, bitshift(st, -13));
        st = uint32(mod(uint64(st) * uint64(hex2dec('5BD1E995')), 2^32));
        st = bitxor(st, bitshift(st, -15));
    end
    h = st;
end

function out = simple_hash_bytes(data)
    % Return hash as 4-byte vector for chaining
    h = simple_hash(data);
    out = uint8([bitshift(h,-24), bitand(bitshift(h,-16),255), ...
                 bitand(bitshift(h,-8),255), bitand(h,255)]);
end

function crc = compute_crc16(bits)
    poly = uint16(hex2dec('1021'));
    crc  = uint16(hex2dec('FFFF'));
    for b = bits
        hi = bitshift(crc, -15);
        crc = bitshift(crc, 1);
        if bitxor(hi, uint16(b))
            crc = bitxor(crc, poly);
        end
    end
end

% =====================================================================
%  BIT-VECTOR UTILITIES
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

% =====================================================================
%  LIVE MONITOR
% =====================================================================
function update_monitor(fig, cycle, status, rx_i, rx_r, snr_i, snr_r, fs)
    if ~ishandle(fig), return; end
    figure(fig);

    % Interrogation waveform
    subplot(3,2,1);
    if ~isempty(rx_i), plot(abs(rx_i)); end
    title(sprintf('Interrogation RX  (Cycle %d)', cycle));
    xlabel('Sample'); ylabel('|I|');

    % Reply waveform
    subplot(3,2,2);
    if ~isempty(rx_r), plot(abs(rx_r)); end
    title(sprintf('Reply RX  (Cycle %d)', cycle));
    xlabel('Sample'); ylabel('|I|');

    % Interrogation spectrum
    subplot(3,2,3);
    if ~isempty(rx_i)
        N = length(rx_i);
        f = (-N/2:N/2-1)*(fs/N);
        plot(f/1e3, 20*log10(abs(fftshift(fft(rx_i)))+eps));
    end
    title('Interrogation Spectrum'); xlabel('kHz'); ylabel('dB');

    % Reply spectrum
    subplot(3,2,4);
    if ~isempty(rx_r)
        N = length(rx_r);
        f = (-N/2:N/2-1)*(fs/N);
        plot(f/1e3, 20*log10(abs(fftshift(fft(rx_r)))+eps));
    end
    title('Reply Spectrum'); xlabel('kHz'); ylabel('dB');

    % Status panel
    subplot(3,2,[5 6]); cla; axis off;
    txt = sprintf('Cycle: %d    Status: %s\nInterrog SNR: %.1f dB    Reply SNR: %.1f dB', ...
        cycle, status, snr_i, snr_r);
    text(0.5, 0.5, txt, 'FontSize', 14, 'FontWeight', 'bold', ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
    if strcmp(status, 'FRIEND')
        set(gca, 'Color', [0.2 0.8 0.2]);
    else
        set(gca, 'Color', [0.8 0.2 0.2]);
    end

    drawnow;
end

% =====================================================================
%  MISC HELPERS
% =====================================================================
function wait_and_continue(fig, interval)
    if ishandle(fig)
        fprintf('  Waiting %d s before retry ...\n', interval);
        pause(interval);
    end
end

function safe_release(varargin)
    for k = 1:length(varargin)
        try
            v = evalin('caller', varargin{k});
            release(v);
        catch
        end
    end
end