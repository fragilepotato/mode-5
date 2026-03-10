%% GENERATE_ADSB_IPCORE  — Build IFF Transponder DSP IP Core for ZedBoard
%
%  Generates Verilog from adsb_ip_top.m using standard MATLAB HDL Coder.
%  No float-to-fixed conversion needed — the DUT uses pure int16/int32
%  arithmetic, so codegen goes straight to HDL.
%
%  Flow:
%    1. Verify toolboxes
%    2. Run test bench to confirm functional correctness
%    3. Configure HDL generation (Verilog, ZedBoard target)
%    4. codegen -config hdl → Verilog in hdl_prj/
%    5. Print Vivado integration steps
%
%  Prerequisites:
%    - MATLAB HDL Coder
%    - MATLAB Coder
%    - Xilinx Vivado 2023.2 on system PATH (for synthesis only)
%
%  Usage:
%    >> cd matlab_codes
%    >> generate_adsb_ipcore
%
clear; clc;

disp('========================================================');
disp('   IFF Transponder DSP — IP Core Generation');
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
    error('Missing toolbox(es): %s\nInstall them before running this script.', ...
        strjoin(missing, ', '));
end
disp('  All required toolboxes found.');

% =====================================================================
%  2. RUN TEST BENCH  (functional verification before synthesis)
% =====================================================================
disp('[2/4] Running test bench (adsb_ip_top_tb) ...');
try
    adsb_ip_top_tb();
    disp('  Test bench completed.');
catch ME
    error('Test bench failed: %s\nFix adsb_ip_top.m before generating HDL.', ...
        ME.message);
end

% =====================================================================
%  3. CONFIGURE HDL GENERATION
%
%  Since adsb_ip_top uses pure integer types (int16/int32/uint8/logical),
%  no float-to-fixed conversion is needed.  codegen goes directly to HDL.
%  This eliminates the MEX compiler dependency entirely.
% =====================================================================
disp('[3/4] Configuring HDL generation ...');

hdlcfg = coder.config('hdl');
hdlcfg.TargetLanguage = 'Verilog';

% Output folder
hdl_output_dir = fullfile(pwd, '..', 'hdl_prj');
if ~exist(hdl_output_dir, 'dir')
    mkdir(hdl_output_dir);
end

disp('  HDL config ready (Verilog, direct integer-to-HDL).');
fprintf('  Output folder: %s\n', hdl_output_dir);

% =====================================================================
%  4. GENERATE VERILOG
%
%  Input types match AD9361 ADC output:
%    rx_re — int16 column vector (6528 x 1)
%    rx_im — int16 column vector (6528 x 1)
%
%  codegen invokes:
%    a) MATLAB Coder: type inference + C code generation
%    b) HDL Coder: C → Verilog RTL
%
%  Output: hdl_prj/adsb_ip_top/hdlsrc/adsb_ip_top.v (+ submodules)
% =====================================================================
disp('[4/4] Generating Verilog HDL ...');
disp('       (Integer DUT — no float-to-fixed step needed)');

FRAME_LEN = 6528;
input_re = coder.typeof(int16(0), [FRAME_LEN 1], [false false]);
input_im = coder.typeof(int16(0), [FRAME_LEN 1], [false false]);

codegen -config hdlcfg adsb_ip_top -args {input_re, input_im} -d hdl_output_dir

disp(' ');
disp('========================================================');
disp('   IP CORE GENERATION COMPLETE');
disp('========================================================');
fprintf('  Verilog output : %s\n', hdl_output_dir);
fprintf('  Top module     : adsb_ip_top\n');
fprintf('  Inputs         : rx_re[15:0], rx_im[15:0]  (int16, 6528 deep)\n');
fprintf('  Outputs        : rx_bits[7:0] x 88, valid_flag (1 bit)\n');
fprintf('  Target device  : xc7z020clg484-1 (ZedBoard)\n');
disp(' ');

% =====================================================================
%  5. NEXT STEPS — Vivado 2023.2 Block Design Integration
% =====================================================================
disp('--- NEXT STEPS (Vivado 2023.2) ---');
disp(' ');
disp('  1. Open Vivado, create project for ZedBoard (xc7z020clg484-1)');
disp('  2. Add generated Verilog files from hdl_prj/ hdlsrc folder');
disp('  3. Package as IP: Tools > Create and Package New IP');
disp('     current project > set vendor/library/name/version');
disp('  4. In Block Design, add:');
disp('       - ZYNQ7 Processing System (PS)');
disp('       - AXI DMA (IQ frames from AD9361 to PL BRAM)');
disp('       - adsb_ip_top (generated IP)');
disp('       - AXI GPIO (ZedBoard LEDs + BTN_CENTER)');
disp('  5. Wire connections:');
disp('       AD9361 I/Q  > AXI DMA > BRAM > adsb_ip_top.rx_re / rx_im');
disp('       adsb_ip_top.rx_bits    > AXI4-Lite register');
disp('       adsb_ip_top.valid_flag > AXI4-Lite register + LED2');
disp('       GPIO BTN_CENTER        > burst trigger');
disp('       GPIO LED0              > RX indicator');
disp('       GPIO LED1              > TX indicator');
disp('  6. Generate Bitstream > export > launch Vitis/SDK');
disp('  7. ARM C firmware:');
disp('       - Configures AD9361 RX on 1030 MHz (12 MSPS)');
disp('       - DMA captures 6528 I/Q samples into BRAM');
disp('       - Triggers adsb_ip_top, reads valid_flag');
disp('       - If valid: reads rx_bits, validates DF=B0 sync');
disp('       - Extracts challenge, builds 8D+ICAO+SQUAWK+PI reply');
disp('       - DMA sends reply via AD9361 TX on 1090 MHz');
disp('       - Toggles LEDs via GPIO');
disp(' ');
disp('========================================================');
