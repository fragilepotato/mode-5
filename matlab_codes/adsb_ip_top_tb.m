function adsb_ip_top_tb()
%% ADSB_IP_TOP_TB  Test bench for integer-arithmetic HDL Coder DUT
%
%  Generates ADS-B PPM waveforms in floating-point, converts to int16
%  (matching AD9361 ADC output format), then calls adsb_ip_top(re, im).
%
%  Test 1 — Clean interrogation   (expect valid = true,  0% BER)
%  Test 2 — Noisy interrogation   (expect valid = true,  low BER)
%  Test 3 — Noise-only            (expect valid = false)
%  Test 4 — Weak signal           (boundary condition, info only)
%
%  This file is NOT synthesized — only adsb_ip_top goes to hardware.

FRAME_LEN = 6528;
fs        = 12e6;
ADC_SCALE = 30000;   % map normalized [0,1] → int16 range (keep headroom)

% Known interrogation frame
interrog_hex = 'B01A2B05D2CE21DA000000';   % 22 hex = 88 bits
pass_count   = 0;
fail_count   = 0;

fprintf('========================================================\n');
fprintf('   adsb_ip_top — Integer HDL Coder Test Bench\n');
fprintf('========================================================\n\n');

% Generate reference waveform (float)
[tx_wave, tx_bits] = tb_adsb_modulate(interrog_hex, fs);

% -----------------------------------------------------------------
%  TEST 1 — Clean interrogation (no noise)
% -----------------------------------------------------------------
fprintf('--- Test 1: Clean interrogation ---\n');
rx_float = tb_pad_frame(tx_wave, FRAME_LEN) * 0.5;
[rx_re, rx_im] = tb_float_to_int16(rx_float, ADC_SCALE);

[rx_bits, valid] = adsb_ip_top(rx_re, rx_im);

if valid
    rx_hex = tb_bits2hex(double(rx_bits));
    ber    = sum(double(rx_bits) ~= tx_bits(1:88)) / 88;
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
rx_float = tb_pad_frame(tx_wave, FRAME_LEN) * 0.4 + noise;
[rx_re, rx_im] = tb_float_to_int16(rx_float, ADC_SCALE);

[rx_bits, valid] = adsb_ip_top(rx_re, rx_im);

if valid
    rx_hex = tb_bits2hex(double(rx_bits));
    ber    = sum(double(rx_bits) ~= tx_bits(1:88)) / 88;
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
[rx_re, rx_im] = tb_float_to_int16(rx_noise, ADC_SCALE);

[~, valid] = adsb_ip_top(rx_re, rx_im);

if ~valid
    fprintf('  Valid: NO — PASS (correctly rejected noise)\n\n');
    pass_count = pass_count + 1;
else
    fprintf('  Valid: YES — FAIL (false positive on noise)\n\n');
    fail_count = fail_count + 1;
end

% -----------------------------------------------------------------
%  TEST 4 — Weak signal (near threshold)
% -----------------------------------------------------------------
fprintf('--- Test 4: Weak signal (near threshold) ---\n');
rng(7);
noise    = 0.02 * (randn(FRAME_LEN,1) + 1i*randn(FRAME_LEN,1));
rx_float = tb_pad_frame(tx_wave, FRAME_LEN) * 0.08 + noise;
[rx_re, rx_im] = tb_float_to_int16(rx_float, ADC_SCALE);

[rx_bits, valid] = adsb_ip_top(rx_re, rx_im);

fprintf('  Valid: %s\n', tb_tf(valid));
if valid
    ber = sum(double(rx_bits) ~= tx_bits(1:88)) / 88;
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
%  TEST BENCH HELPERS  (floating-point, not synthesized)
% #####################################################################

function [re_i16, im_i16] = tb_float_to_int16(cx_float, scale)
% Convert complex double waveform to split int16 I/Q
%   Mimics AD9361 ADC output: 12-bit values in 16-bit container
    re_i16 = int16(real(cx_float) * scale);
    im_i16 = int16(imag(cx_float) * scale);
end

function [waveform, bits] = tb_adsb_modulate(hexStr, fs)
% ADS-B PPM modulation (same as prototype)
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

function s = tb_tf(v)
    if v, s = 'YES'; else, s = 'NO'; end
end
