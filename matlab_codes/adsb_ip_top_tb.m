function adsb_ip_top_tb()
%% ADSB_IP_TOP_TB  Streaming test bench — feeds samples one at a time
%
%  Generates ADS-B PPM IQ waveforms, quantizes to int16, then calls
%  adsb_ip_top once per sample (simulating the hardware clock).
%  Collects decoded bits from bit_valid strobes and verifies them.
%
%  Test 1 — Clean interrogation   (expect 0% BER)
%  Test 2 — Noisy interrogation   (expect low BER)
%  Test 3 — Noise-only            (expect no frame_valid)
%  Test 4 — Weak signal           (boundary, info only)

FRAME_LEN = 6528;
fs        = 12e6;
ADC_SCALE = 30000;

interrog_hex = 'B01A2B05D2CE21DA000000';
pass_count   = 0;
fail_count   = 0;

fprintf('========================================================\n');
fprintf('   adsb_ip_top — Streaming HDL Test Bench\n');
fprintf('========================================================\n\n');

[tx_wave, tx_bits] = tb_adsb_modulate(interrog_hex, fs);

% -----------------------------------------------------------------
%  TEST 1 — Clean interrogation
% -----------------------------------------------------------------
fprintf('--- Test 1: Clean interrogation ---\n');
rx_float = tb_pad_frame(tx_wave, FRAME_LEN) * 0.5;
[decoded, got_frame, got_valid] = tb_run_streaming(rx_float, ADC_SCALE, FRAME_LEN);

if got_frame && got_valid
    rx_hex = tb_bits2hex(decoded);
    n = min(length(decoded), 88);
    ber = sum(decoded(1:n) ~= tx_bits(1:n)) / 88;
    fprintf('  Frame    : YES (valid)\n');
    fprintf('  Decoded  : %s\n', rx_hex);
    fprintf('  Expected : %s\n', interrog_hex);
    fprintf('  Bits     : %d\n', length(decoded));
    fprintf('  BER      : %.1f%%\n', ber*100);
    if ber == 0
        fprintf('  >> PASS\n\n');
        pass_count = pass_count + 1;
    else
        fprintf('  >> FAIL (BER > 0)\n\n');
        fail_count = fail_count + 1;
    end
elseif got_frame && ~got_valid
    fprintf('  Frame: YES but NOT valid (SNR fail) — FAIL\n\n');
    fail_count = fail_count + 1;
else
    fprintf('  Frame: NO — FAIL (no frame_done pulse)\n\n');
    fail_count = fail_count + 1;
end

% -----------------------------------------------------------------
%  TEST 2 — Noisy interrogation (SNR ~20 dB)
% -----------------------------------------------------------------
fprintf('--- Test 2: Noisy interrogation (SNR ~20 dB) ---\n');
rng(42);
noise    = 0.005 * (randn(FRAME_LEN,1) + 1i*randn(FRAME_LEN,1));
rx_float = tb_pad_frame(tx_wave, FRAME_LEN) * 0.4 + noise;
[decoded, got_frame, got_valid] = tb_run_streaming(rx_float, ADC_SCALE, FRAME_LEN);

if got_frame && got_valid
    rx_hex = tb_bits2hex(decoded);
    n = min(length(decoded), 88);
    ber = sum(decoded(1:n) ~= tx_bits(1:n)) / 88;
    fprintf('  Frame    : YES (valid)\n');
    fprintf('  Decoded  : %s\n', rx_hex);
    fprintf('  BER      : %.1f%%\n', ber*100);
    if ber < 0.05
        fprintf('  >> PASS (BER < 5%%)\n\n');
        pass_count = pass_count + 1;
    else
        fprintf('  >> FAIL (BER too high)\n\n');
        fail_count = fail_count + 1;
    end
else
    fprintf('  Frame result: frame=%d valid=%d — FAIL\n\n', got_frame, got_valid);
    fail_count = fail_count + 1;
end

% -----------------------------------------------------------------
%  TEST 3 — Noise-only
% -----------------------------------------------------------------
fprintf('--- Test 3: Noise-only ---\n');
rng(99);
rx_noise = 0.01 * (randn(FRAME_LEN,1) + 1i*randn(FRAME_LEN,1));
[~, got_frame, got_valid] = tb_run_streaming(rx_noise, ADC_SCALE, FRAME_LEN);

if ~got_valid
    fprintf('  Valid: NO — PASS (correctly rejected noise)\n\n');
    pass_count = pass_count + 1;
else
    fprintf('  Valid: YES — FAIL (false positive on noise)\n\n');
    fail_count = fail_count + 1;
end

% -----------------------------------------------------------------
%  TEST 4 — Weak signal
% -----------------------------------------------------------------
fprintf('--- Test 4: Weak signal (near threshold) ---\n');
rng(7);
noise    = 0.02 * (randn(FRAME_LEN,1) + 1i*randn(FRAME_LEN,1));
rx_float = tb_pad_frame(tx_wave, FRAME_LEN) * 0.08 + noise;
[decoded, got_frame, got_valid] = tb_run_streaming(rx_float, ADC_SCALE, FRAME_LEN);

fprintf('  Frame: %s,  Valid: %s\n', tb_tf(got_frame), tb_tf(got_valid));
if got_valid && ~isempty(decoded)
    n = min(length(decoded), 88);
    ber = sum(decoded(1:n) ~= tx_bits(1:n)) / 88;
    fprintf('  BER      : %.1f%%\n', ber*100);
end
fprintf('  >> INFO (boundary condition)\n\n');
pass_count = pass_count + 1;

% -----------------------------------------------------------------
%  SUMMARY
% -----------------------------------------------------------------
fprintf('========================================================\n');
fprintf('  Results: %d PASS, %d FAIL (out of 4 tests)\n', ...
    pass_count, fail_count);
fprintf('========================================================\n');

end

% #####################################################################
%  STREAMING RUNNER — feeds samples one at a time to the DUT
% #####################################################################

function [decoded_bits, got_frame, got_valid] = tb_run_streaming(rx_float, adc_scale, frame_len)
% Simulate hardware clock: call adsb_ip_top once per sample
    re_samples = int16(real(rx_float) * adc_scale);
    im_samples = int16(imag(rx_float) * adc_scale);

    decoded_bits = [];
    got_frame = false;
    got_valid = false;

    % Clear persistent state by calling with a fresh MATLAB function context
    clear adsb_ip_top;

    for i = 1:frame_len
        [bit_out, bit_valid, frame_done, frame_valid] = ...
            adsb_ip_top(re_samples(i), im_samples(i), true);

        if bit_valid
            decoded_bits = [decoded_bits, double(bit_out)]; %#ok<AGROW>
        end
        if frame_done
            got_frame = true;
            got_valid = frame_valid;
        end
    end
end

% #####################################################################
%  TEST BENCH HELPERS  (floating-point, not synthesized)
% #####################################################################

function [waveform, bits] = tb_adsb_modulate(hexStr, fs)
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
