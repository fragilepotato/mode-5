%% GENERATE_ADSB_IPCORE  — Build streaming IFF Transponder IP Core
%
%  Generates Verilog from adsb_ip_top.m (sample-at-a-time architecture).
%  Pure int16/int32/uint8/logical — no float, no MEX needed.
%
%  DUT interface (per clock cycle):
%    Inputs:  re_in (int16), im_in (int16), sample_valid (logical)
%    Outputs: bit_out (uint8), bit_valid (logical),
%             frame_done (logical), frame_valid (logical)
%    Total IO: 44 pins (well under 5000 threshold)
%
%  Prerequisites:
%    - MATLAB HDL Coder
%    - MATLAB Coder
%
%  Usage:
%    >> cd matlab_codes
%    >> generate_adsb_ipcore
%
clear; clc;

disp('========================================================');
disp('   IFF Transponder DSP — Streaming IP Core Generation');
disp('   Target: ZedBoard xc7z020clg484-1, Vivado 2023.2');
disp('========================================================');

% =====================================================================
%  1. VERIFY TOOLBOXES
% =====================================================================
disp('[1/4] Checking required toolboxes ...');

required = {'MATLAB Coder', 'HDL Coder'};
installed = ver;
installed_names = {installed.Name};
missing = {};
for i = 1:numel(required)
    if ~any(strcmpi(installed_names, required{i}))
        missing{end+1} = required{i}; %#ok<AGROW>
    end
end
if ~isempty(missing)
    error('Missing toolbox(es): %s', strjoin(missing, ', '));
end
disp('  All required toolboxes found.');

% =====================================================================
%  2. RUN TEST BENCH
% =====================================================================
disp('[2/4] Running streaming test bench ...');
try
    adsb_ip_top_tb();
    disp('  Test bench completed.');
catch ME
    error('Test bench failed: %s', ME.message);
end

% =====================================================================
%  3. CONFIGURE HDL
% =====================================================================
disp('[3/4] Configuring HDL generation ...');

hdlcfg = coder.config('hdl');
hdlcfg.TargetLanguage = 'Verilog';

hdl_output_dir = fullfile(pwd, '..', 'hdl_prj');
if ~exist(hdl_output_dir, 'dir')
    mkdir(hdl_output_dir);
end

fprintf('  Output folder: %s\n', hdl_output_dir);

% =====================================================================
%  4. GENERATE VERILOG
%
%  Streaming inputs: one int16 I, one int16 Q, one logical valid
%  per clock cycle.  Persistent state inside the function maps to
%  registers and block RAM in the generated RTL.
% =====================================================================
disp('[4/4] Generating Verilog (streaming, sample-at-a-time) ...');

input_re    = coder.typeof(int16(0));       % scalar int16
input_im    = coder.typeof(int16(0));       % scalar int16
input_valid = coder.typeof(false);          % scalar logical

codegen -config hdlcfg adsb_ip_top ...
    -args {input_re, input_im, input_valid} ...
    -d hdl_output_dir

disp(' ');
disp('========================================================');
disp('   IP CORE GENERATION COMPLETE');
disp('========================================================');
fprintf('  Verilog output  : %s\n', hdl_output_dir);
fprintf('  Top module      : adsb_ip_top\n');
disp('  Inputs (per clk): re_in[15:0], im_in[15:0], sample_valid');
disp('  Outputs(per clk): bit_out[7:0], bit_valid, frame_done, frame_valid');
fprintf('  Target device   : xc7z020clg484-1 (ZedBoard)\n');
disp(' ');

disp('--- NEXT STEPS (Vivado 2023.2) ---');
disp(' ');
disp('  1. Open Vivado, create project for ZedBoard (xc7z020clg484-1)');
disp('  2. Add Verilog files from hdl_prj/ hdlsrc folder');
disp('  3. Package as IP: Tools > Create and Package New IP');
disp('  4. In Block Design:');
disp('       - ZYNQ7 Processing System (PS)');
disp('       - AD9361 LVDS interface IP (or ADI HDL reference design)');
disp('       - adsb_ip_top (your generated streaming IP)');
disp('       - AXI GPIO (LEDs + BTN_CENTER)');
disp('  5. Wire AD9361 I/Q samples directly to adsb_ip_top ports:');
disp('       ad9361_data_i[15:0] > re_in');
disp('       ad9361_data_q[15:0] > im_in');
disp('       ad9361_data_valid   > sample_valid');
disp('       bit_out + bit_valid > shift register (88-bit frame assembler)');
disp('       frame_done          > interrupt to ARM PS');
disp('       frame_valid         > LED2');
disp('  6. ARM firmware:');
disp('       - On frame_done interrupt: read 88-bit buffer from PL register');
disp('       - Validate DF=B0, extract challenge');
disp('       - Build reply [8D+ICAO+SQUAWK+PI], DMA to AD9361 TX on 1090 MHz');
disp(' ');
disp('========================================================');
