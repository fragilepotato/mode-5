function adsb_ip_top_tb()
%% ADSB_IP_TOP_TB  Test bench for HDL Coder float-to-fixed range analysis
%
%  Exercises adsb_ip_top with:
%    Test 1 — Clean interrogation frame  (expect valid_flag = true, 0% BER)
%    Test 2 — Noisy interrogation frame  (expect valid_flag = true, low BER)
%    Test 3 — Noise-only frame           (expect valid_flag = false)
%    Test 4 — Weak-signal frame          (SNR near threshold)
%
%  This file is consumed by HDL Coder's -float2fixed pipeline to
%  determine the dynamic range of every internal variable in adsb_ip_top.
%  It is NOT synthesized — only the DUT (adsb_ip_top) goes to hardware.

FRAME_LEN = 6528;
fs        = 12e6;

% Known interrogation: [DF=B0][ID=1A2B][Mode=05][Challenge=D2CE21DA][CRC=000000]
interrog_hex = 'B01A2B05D2CE21DA000000';   % 22 hex = 88 bits
pass_count   = 0;
fail_count   = 0;

fprintf('========================================================\n');
fprintf('   adsb_ip_top — HDL Coder Test Bench\n');
fprintf('========================================================\n\n');

% -----------------------------------------------------------------
%  TEST 1 — Clean interrogation (no noise)
% -----------------------------------------------------------------
fprintf('--- Test 1: Clean interrogation ---\n');
[tx_wave, tx_bits] = tb_adsb_modulate(interrog_hex, fs);
rx_frame = tb_pad_frame(tx_wave, FRAME_LEN) * 0.5;

[rx_bits, valid] = adsb_ip_top(rx_frame);

if valid
    rx_hex = tb_bits2hex(rx_bits);
    ber    = sum(rx_bits ~= tx_bits(1:88)) / 88;
    fprintf('  Valid   : YES\n');
    fprintf('  Decoded : %s\n', rx_hex);
    fprintf('  Expected: %s\n', interrog_hex);
    fprintf('  BER     : %.1f%%\n', ber*100);
    if ber == 0
        fprintf('  >> PASS\n\n');
        pass_count = pass_count + 1;
    else
        fprintf('  >> FAIL (BER > 0)\n\n');
        fail_count = fail_count + 1;
    end
else
    fprintf('  Valid: NO — FAIL (should have detected preamble)\n\n');
    fail_count = fail_count + 1;
end

% -----------------------------------------------------------------
%  TEST 2 — Noisy interrogation (SNR ~20 dB)
% -----------------------------------------------------------------
fprintf('--- Test 2: Noisy interrogation (SNR ~20 dB) ---\n');
rng(42);
noise    = 0.005 * (randn(FRAME_LEN,1) + 1i*randn(FRAME_LEN,1));
rx_frame = tb_pad_frame(tx_wave, FRAME_LEN) * 0.4 + noise;

[rx_bits, valid] = adsb_ip_top(rx_frame);

if valid
    rx_hex = tb_bits2hex(rx_bits);
    ber    = sum(rx_bits ~= tx_bits(1:88)) / 88;
    fprintf('  Valid   : YES\n');
    fprintf('  Decoded : %s\n', rx_hex);
    fprintf('  BER     : %.1f%%\n', ber*100);
    if ber < 0.05
        fprintf('  >> PASS (BER < 5%%)\n\n');
        pass_count = pass_count + 1;
    else
        fprintf('  >> FAIL (BER too high)\n\n');
        fail_count = fail_count + 1;
    end
else
    fprintf('  Valid: NO — FAIL\n\n');
    fail_count = fail_count + 1;
end

% -----------------------------------------------------------------
%  TEST 3 — Noise-only (no signal)
% -----------------------------------------------------------------
fprintf('--- Test 3: Noise-only ---\n');
rng(99);
rx_noise = 0.01 * (randn(FRAME_LEN,1) + 1i*randn(FRAME_LEN,1));

[~, valid] = adsb_ip_top(rx_noise);

if ~valid
    fprintf('  Valid: NO — PASS (correctly rejected noise)\n\n');
    pass_count = pass_count + 1;
else
    fprintf('  Valid: YES — FAIL (false positive on noise)\n\n');
    fail_count = fail_count + 1;
end

% -----------------------------------------------------------------
%  TEST 4 — Weak signal (SNR near threshold)
% -----------------------------------------------------------------
fprintf('--- Test 4: Weak signal (near threshold) ---\n');
rng(7);
noise    = 0.02 * (randn(FRAME_LEN,1) + 1i*randn(FRAME_LEN,1));
rx_frame = tb_pad_frame(tx_wave, FRAME_LEN) * 0.08 + noise;

[rx_bits, valid] = adsb_ip_top(rx_frame);

fprintf('  Valid: %s\n', tf_str(valid));
if valid
    ber = sum(rx_bits ~= tx_bits(1:88)) / 88;
    fprintf('  BER  : %.1f%%\n', ber*100);
end
fprintf('  >> INFO (boundary condition — either result acceptable)\n\n');
pass_count = pass_count + 1;  % boundary: no hard pass/fail

% -----------------------------------------------------------------
%  SUMMARY
% -----------------------------------------------------------------
fprintf('========================================================\n');
fprintf('  Results: %d PASS, %d FAIL (out of 4 tests)\n', ...
    pass_count, fail_count);
fprintf('========================================================\n');

end

% #####################################################################
%  TEST BENCH HELPERS  (not synthesized)
% #####################################################################

function [waveform, bits] = tb_adsb_modulate(hexStr, fs)
% Same modulation as prototype — generates ADS-B PPM waveform
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

function frame = tb_pad_frame(tx_wave, frame_len)
% Pad or trim waveform to exactly frame_len samples (column vector)
    n = length(tx_wave);
    if n >= frame_len
        frame = tx_wave(1:frame_len);
    else
        frame = [tx_wave; zeros(frame_len - n, 1)];
    end
end

function h = tb_bits2hex(bits)
    chars = '0123456789ABCDEF';
    h = '';
    for i = 1:4:length(bits)-3
        v = bits(i)*8 + bits(i+1)*4 + bits(i+2)*2 + bits(i+3);
        h = [h, chars(v+1)]; %#ok<AGROW>
    end
end

function s = tf_str(v)
    if v, s = 'YES'; else, s = 'NO'; end
end
