%% ZEDBOARD / PLUTO ADS-B MODEM (TX & RX DECODER)
clear all; close all; clc;

% --- 1. CONFIGURATION ---
% Hardware Toggle: Set to 'ZedBoard' or 'Pluto'
hw_target = 'Pluto'; 

ip_zed = '192.168.1.10';    % ZedBoard IP
ip_pluto = 'ip:192.168.2.1';         % Default Pluto USB context

fs = 12e6;                  % 12 MSPS (Standard for ADS-B decoding)
fc = 1090e6;                % 1090 MHz Center Frequency
msg_hex = '8D4840D6202CC371C32CE0576098'; % ADS-B Hex String

% --- 2. MESSAGE ENCODING (HEX -> PPM WAVEFORM) ---
disp(['1. Encoding Message: ' msg_hex]);
[tx_wave, bits_sent] = local_ADS_B_Modulator(msg_hex, fs);

% --- 3. HARDWARE SETUP (CROSS-DEVICE) ---
disp('2. Initializing Pluto TX and ZedBoard RX...');

ip_zed = '192.168.1.10';    
ip_pluto = 'ip:192.168.2.1'; 

try
    % 1. Transmitter - Pluto
    tx = sdrtx('Pluto', 'RadioID', ip_pluto, 'CenterFrequency', fc, ...
               'BasebandSampleRate', fs);
    % CRITICAL: Drop TX gain to minimum for direct SMA cable connection 
    % to prevent blowing out the AD9361 receiver front-end.
    tx.Gain = -60; 
               
    % 2. Receiver - ZedBoard (AD9361)
    rx = sdrrx('AD936x', 'IPAddress', ip_zed, 'CenterFrequency', fc, ...
               'BasebandSampleRate', fs, 'SamplesPerFrame', 2e5, ...
               'OutputDataType', 'double', 'ChannelMapping', 1);
    % CRITICAL: Drop RX gain to 0 for direct SMA cable connection
    rx.GainSource = 'Manual';
    rx.Gain = 0; 
    
catch ME
    error('Radio Connection Failed. Error: %s', ME.message);
end

% --- 4. TRANSMIT & CAPTURE ---
disp('3. Transmitting & Capturing...');
transmitRepeat(tx, tx_wave); % Start continuous TX
pause(0.5);                  % Wait for TX to stabilize
data_rx = rx();              % Capture one snapshot
release(tx); release(rx);    % Stop radio immediately

% --- 5. SOFTWARE DECODING ---
disp('4. Attempting to Decode...');

% A. Magnitude & Noise Floor
mag = abs(data_rx);
mag = mag - mean(mag); % Remove DC offset

% B. Preamble Detection (Correlation)
% ADS-B Preamble: Pulses at 0, 1.0, 3.5, 4.5 microseconds
samplesPerUs = fs / 1e6;
pulse_width  = round(0.5 * samplesPerUs);

% Create Ideal Preamble Kernel for Correlation
kernel = zeros(round(8 * samplesPerUs), 1);
idx_p = round([0, 1, 3.5, 4.5] * samplesPerUs) + 1;
for i = idx_p
    kernel(i : i + pulse_width - 1) = 1;
end
kernel = kernel * 2 - 1; % Make it bipolar (-1 to +1) for sharp correlation

% Run Correlation
[c, lags] = xcorr(mag, kernel);
[peak_val, peak_idx_lag] = max(c);
start_idx = lags(peak_idx_lag);

% C. Demodulate Bits (PPM)
% Dynamic threshold based on peak (adjust as needed for Pluto vs ZedBoard)
threshold = 0.4 * max(c); 

if peak_val > threshold 
    disp('   > Preamble Detected!');
    
    % Jump to start of data (8us after preamble start)
    data_start = start_idx + round(8 * samplesPerUs);
    
    bits_rx = [];
    samplesPerBit = round(1 * samplesPerUs);
    
    % Loop through 112 bits (ADS-B message length)
    for k = 0:111
        idx = data_start + (k * samplesPerBit);
        if (idx + samplesPerBit) > length(mag), break; end
        
        chunk = mag(idx : idx + samplesPerBit - 1);
        
        % Split into first half and second half
        half = floor(length(chunk)/2);
        power_first = sum(chunk(1:half));
        power_second = sum(chunk(half+1:end));
        
        % Decision: PPM Logic
        if power_first > power_second
            bits_rx(end+1) = 1;
        else
            bits_rx(end+1) = 0;
        end
    end
    
    % D. Convert Bits back to Hex
    hex_rx = local_BitsToHex(bits_rx);
    disp('------------------------------------------------');
    disp(['SENT:     ' msg_hex]);
    disp(['RECEIVED: ' hex_rx]);
    disp('------------------------------------------------');
    
    if strcmp(msg_hex, hex_rx)
        disp('RESULT: SUCCESS. Perfect Match.');
    else
        disp('RESULT: Mismatch (Check Gain or Noise).');
        if length(bits_sent) == length(bits_rx)
            disp(['Bit Errors: ' num2str(sum(abs(bits_sent - bits_rx)))]);
        else
            disp('Bit length mismatch. Packet truncated.');
        end
    end
    
    % Plot
    figure;
    subplot(2,1,1); 
    plot_end = min(start_idx+2000, length(mag));
    plot(mag(start_idx:plot_end)); 
    title('Received Raw Signal (Zoomed at Packet)'); grid on;
    subplot(2,1,2); plot(c); title('Correlation Peak (Preamble Detection)');
    
else
    disp('FAILED: Preamble not found. Signal too weak or missing?');
end

% --- LOCAL FUNCTIONS ---
function [waveform, bits] = local_ADS_B_Modulator(hexStr, fs)
    % Convert Hex to Bits
    bits = [];
    for i = 1:length(hexStr)
        val = hex2dec(hexStr(i));
        bits = [bits, bitget(val, 4:-1:1)];
    end
    
    T_bit = 1e-6; % 1 microsecond per bit
    sps = round(fs * T_bit); % Samples per bit
    
    % 1. Create Preamble (8us)
    preamble_len = round(8 * sps);
    preamble_sig = zeros(1, preamble_len);
    pulse_len = round(0.5 * sps);
    idx_list = round([0, 1, 3.5, 4.5] * sps) + 1;
    
    for k = idx_list
        preamble_sig(k : k+pulse_len-1) = 1;
    end
    
    % 2. Create Data (PPM Modulation)
    data_sig = [];
    for b = bits
        pat = zeros(1, sps);
        mid = floor(sps/2);
        if b == 1
            pat(1:mid) = 1; % Pulse first
        else
            pat(mid+1:end) = 1; % Pulse second
        end
        data_sig = [data_sig, pat];
    end
    
    % Combine and Pad
    waveform = [preamble_sig, data_sig, zeros(1, 1000)]; % 1000 samples silence
    waveform = complex(double(waveform.'), 0); % Format for SDR TX
end

function hexStr = local_BitsToHex(bits)
    hexStr = '';
    remBits = mod(length(bits), 4);
    if remBits > 0
        bits = [bits, zeros(1, 4-remBits)];
    end
    
    chars = '0123456789ABCDEF';
    for i = 1:4:length(bits)
        chunk = bits(i:i+3);
        val = chunk(1)*8 + chunk(2)*4 + chunk(3)*2 + chunk(4)*1;
        hexStr = [hexStr, chars(val+1)];
    end
end