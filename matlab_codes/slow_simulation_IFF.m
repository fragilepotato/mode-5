%% ZEDBOARD ROBUST ADS-B DECODER (With DC Correction)
% Hardware: ZedBoard + FMCOMMS3 (AD9361)
% Fixes: Adds High-Pass Filter and improved sync to handle DC Ramp.
clear all; close all; instrreset;

% --- 1. PARAMETERS ---
dev_name = 'AD936x';       
ip_addr  = '192.168.1.10'; 
fs       = 4e6;            % 4 MSPS 
fc       = 1090e6;         

% --- 2. GENERATE STATIC MESSAGE ---
% We use a fixed message so we can debug easily.
% HEX: AA BB CC DD ...
tx_hex_fixed = 'AABBCCDDEEFF0011223344556677';
tx_bits = hexToBinaryVector(tx_hex_fixed, 112);

% Modulate (PPM)
modulated_data = [];
for b = tx_bits
    if b == 1
        modulated_data = [modulated_data, 1 1 0 0]; 
    else
        modulated_data = [modulated_data, 0 0 1 1]; 
    end
end

% Preamble (standard ADS-B)
preamble_vec = [1 1 0 0 1 1 0 0 0 0 0 0 0 0 1 1 0 0 1 1 0 0]; 
header_padding = zeros(1, 10); % To make it exactly 8us (32 samples)
packet_full = [preamble_vec, header_padding, modulated_data];

% TX Waveform
tx_wave = complex(double([packet_full, zeros(1, 2000)].'), 0);

% --- 3. RADIO CONFIGURATION ---
disp(['Connecting to ' ip_addr '...']);
try
    tx = sdrtx(dev_name, 'IPAddress', ip_addr, ...
               'CenterFrequency', fc, 'BasebandSampleRate', fs, ...
               'ChannelMapping', 1);
           
    rx = sdrrx(dev_name, 'IPAddress', ip_addr, ...
               'CenterFrequency', fc, 'BasebandSampleRate', fs, ...
               'SamplesPerFrame', length(tx_wave), ...
               'OutputDataType', 'double', 'ChannelMapping', 1);
           
    rx.GainSource = 'Manual';
    rx.Gain = 60; 
catch
    error('Hardware Error. Reboot ZedBoard.');
end

% --- 4. VISUALIZATION ---
figure('Name', 'Decoder Debug', 'Position', [100 100 800 600]);
subplot(3,1,1); h_raw = plot(nan, 'y'); title('Step 1: Raw RX (Noisy)'); axis tight;
subplot(3,1,2); h_filt = plot(nan, 'c'); title('Step 2: Filtered & Flattened'); axis tight;
subplot(3,1,3); h_bits = stem(nan, 'Filled'); title('Step 3: Decoded Bits (First 50)'); ylim([-0.2 1.2]);

% --- 5. DECODING LOOP ---
disp('------------------------------------------------');
disp(['TX TARGET: ' tx_hex_fixed]); 
disp('------------------------------------------------');

testRunning = true;
cleanupObj = onCleanup(@() cleanupRadio(tx, rx));

while testRunning && ishandle(gcf)
    % 1. Transmit
    tx(tx_wave);
    
    % 2. Receive
    data_rx = rx();
    
    % 3. CRITICAL FIX: High-Pass Filter
    % This removes the "Ramp" so the decision threshold works
    try
        rx_flat = highpass(data_rx, 100e3, fs);
    catch
        % Fallback manual filter (Difference)
        rx_flat = [0; diff(data_rx)]; 
    end
    
    % Take Magnitude and Normalize
    rx_amp  = abs(rx_flat);
    rx_norm = rx_amp / max(rx_amp);
    
    % 4. SYNC (Correlation)
    % Find where the preamble starts
    % We construct the header to correlate against
    header_template = [preamble_vec, header_padding];
    [c, lags] = xcorr(rx_norm, header_template);
    [max_corr, max_idx] = max(c);
    
    % Calculate start index
    start_idx = lags(max_idx);
    
    % Threshold: Only decode if correlation is strong (> 0.6)
    if max_corr > 0.6 && start_idx > 0 && (start_idx + length(packet_full)) < length(rx_norm)
        
        % 5. DEMODULATE
        rx_bits = zeros(1, 112);
        data_start = start_idx + 32; % 32 samples = 8us header
        
        for i = 1:112
            idx = data_start + (i-1)*4;
            
            % Robust Sampling: Use middle 2 samples to avoid edge jitter
            % Pulse is samples 1,2. Gap is 3,4.
            % We sum all 4 to get total energy balance.
            
            chunk = rx_norm(idx : idx+3);
            e_first  = chunk(1) + chunk(2); % Energy in first half
            e_second = chunk(3) + chunk(4); % Energy in second half
            
            if e_first > e_second
                rx_bits(i) = 1;
            else
                rx_bits(i) = 0;
            end
        end
        
        % 6. CHECK ERRORS
        bit_errors = sum(abs(tx_bits - rx_bits));
        rx_hex_str = binaryVectorToHex(rx_bits);
        
        if bit_errors == 0
            fprintf('RX: %s [PERFECT MATCH]\n', rx_hex_str);
        else
            fprintf('RX: %s [Errors: %d/112]\n', rx_hex_str, bit_errors);
        end
        
        % Update Plots
        set(h_raw, 'YData', abs(data_rx));
        set(h_filt, 'YData', rx_norm);
        set(h_bits, 'XData', 1:50, 'YData', rx_bits(1:50));
        drawnow;
        
    else
        % fprintf('.'); % Searching...
    end
end

% --- HELPER FUNCTIONS ---
function cleanupRadio(txObj, rxObj)
    if exist('txObj', 'var'), release(txObj); end
    if exist('rxObj', 'var'), release(rxObj); end
end

function hexStr = binaryVectorToHex(binVec)
    hexStr = '';
    hexMap = '0123456789ABCDEF';
    for k = 1:4:length(binVec)
        chunk = binVec(k:k+3);
        val = 8*chunk(1) + 4*chunk(2) + 2*chunk(3) + 1*chunk(4);
        hexStr = [hexStr, hexMap(val+1)];
    end
end