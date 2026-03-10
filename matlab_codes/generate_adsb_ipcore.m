%% GENERATE_ADSB_IPCORE  — Build IFF Transponder DSP IP Core for ZedBoard
%
%  Equivalent of the dlhdl.buildProcessor / dlhdl.Workflow script you used
%  for the deep-learning model, but targeting the traditional DSP pipeline
%  (adsb_preprocess → adsb_framesync → adsb_decode_bits) via standard
%  MATLAB HDL Coder instead of Deep Learning HDL Toolbox.
%
%  Flow:
%    1. Verify toolboxes
%    2. Run test bench to confirm functional correctness
%    3. Configure float-to-fixed conversion (auto range analysis)
%    4. Configure HDL generation for ZedBoard xc7z020, Vivado 2023.2
%    5. codegen  -float2fixed  -config hdl  →  Verilog IP in hdl_prj/
%    6. Print next steps for Vivado block design integration
%
%  Prerequisites:
%    - MATLAB HDL Coder (hdlcoder)
%    - Fixed-Point Designer (fixedpoint)
%    - Xilinx Vivado 2023.2 on system PATH
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
disp('[1/5] Checking required toolboxes ...');

required = {'MATLAB Coder', 'HDL Coder', 'Fixed-Point Designer'};
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
disp('[2/5] Running test bench (adsb_ip_top_tb) ...');
try
    adsb_ip_top_tb();
    disp('  Test bench completed.');
catch ME
    error('Test bench failed: %s\nFix adsb_ip_top.m before generating HDL.', ...
        ME.message);
end

% =====================================================================
%  3. CONFIGURE FLOAT-TO-FIXED CONVERSION
%
%  HDL Coder runs the test bench to collect min/max ranges for every
%  internal variable, then proposes fixed-point types automatically.
%  DefaultWordLength = 18 matches the DSP48E1 slices on Zynq-7020.
% =====================================================================
disp('[3/5] Configuring float-to-fixed conversion ...');

fxptcfg = coder.config('fixpt');
fxptcfg.TestBenchName      = 'adsb_ip_top_tb';
fxptcfg.DefaultWordLength  = 18;     % DSP48E1 native width on Zynq
fxptcfg.TestNumerics       = true;   % verify fixed-point matches float
fxptcfg.LogIOForComparisonPlotting = true;
fxptcfg.DefaultFractionLength = 14; % 18.14 gives ±8 range with 14-bit frac

disp('  Fixed-point config ready (18-bit word, auto fraction length).');

% =====================================================================
%  4. CONFIGURE HDL GENERATION
%
%  This is the direct equivalent of your dlhdl reference:
%    dlhdl.ProcessorConfig  →  coder.config('hdl')
%    hPC.SynthesisToolChipFamily = 'Zynq'  →  hdlcfg.SynthesisToolChipFamily
%    dlhdl.buildProcessor   →  codegen -float2fixed ... -config hdlcfg
% =====================================================================
disp('[4/5] Configuring HDL generation ...');

hdlcfg = coder.config('hdl');

% --- Target language ---
hdlcfg.TargetLanguage = 'Verilog';

% --- Synthesis tool + device (ZedBoard) ---
hdlcfg.SynthesisTool           = 'Xilinx Vivado';
hdlcfg.SynthesisToolChipFamily = 'Zynq';
hdlcfg.SynthesisToolDeviceName = 'xc7z020';
hdlcfg.SynthesisToolPackageName = 'clg484';
hdlcfg.SynthesisToolSpeedValue  = '-1';

% --- Output folder ---
hdlcfg.TargetDirectory = fullfile(pwd, '..', 'hdl_prj', 'hdlsrc');

% --- Reports + optimizations ---
hdlcfg.GenerateReport        = true;
hdlcfg.GenerateHDLTestBench  = true;   % Verilog testbench for ModelSim/Vivado sim
hdlcfg.OptimizeForTiming     = true;
hdlcfg.InputPipelining       = true;
hdlcfg.OutputPipelining      = true;

disp('  HDL config ready (Verilog, xc7z020clg484-1).');

% =====================================================================
%  5. GENERATE HDL
%
%  codegen performs:
%    a) Float-to-fixed range analysis (runs test bench)
%    b) Fixed-point code generation
%    c) HDL (Verilog) code generation
%    d) Optional synthesis report
%
%  Output: hdl_prj/hdlsrc/adsb_ip_top.v  (+ submodules)
% =====================================================================
disp('[5/5] Generating fixed-point HDL — this may take a few minutes ...');
disp('       (Running range analysis → fixed-point → Verilog pipeline)');

% Input specification: complex double column vector, 6528 x 1
FRAME_LEN = 6528;
input_type = {complex(zeros(FRAME_LEN, 1))};

codegen -float2fixed fxptcfg -config hdlcfg adsb_ip_top -args input_type

disp(' ');
disp('========================================================');
disp('   IP CORE GENERATION COMPLETE');
disp('========================================================');
fprintf('  Verilog output : %s\n', fullfile(pwd, '..', 'hdl_prj', 'hdlsrc'));
fprintf('  Top module     : adsb_ip_top\n');
fprintf('  Target device  : xc7z020clg484-1 (ZedBoard)\n');
disp(' ');

% =====================================================================
%  6. NEXT STEPS — Vivado 2023.2 Block Design Integration
% =====================================================================
disp('--- NEXT STEPS (Vivado 2023.2) ---');
disp(' ');
disp('  1. Open Vivado, create project for ZedBoard (xc7z020clg484-1)');
disp('  2. Add generated Verilog files from hdl_prj/hdlsrc/');
disp('  3. Package as IP: Tools → Create and Package New IP → Package');
disp('     current project → set vendor/library/name/version');
disp('  4. In Block Design, add:');
disp('       - ZYNQ7 Processing System (PS)');
disp('       - AXI DMA (for IQ frame transfer from AD9361 to PL BRAM)');
disp('       - adsb_ip_top (your generated IP)');
disp('       - AXI GPIO (connect to ZedBoard LEDs + center button)');
disp('  5. Wire connections:');
disp('       AD9361 IQ → AXI DMA → BRAM → adsb_ip_top.rx_iq');
disp('       adsb_ip_top.rx_bits      → AXI4-Lite register');
disp('       adsb_ip_top.valid_flag   → AXI4-Lite register + LED2');
disp('       GPIO BTN_CENTER          → burst trigger');
disp('       GPIO LED0                → RX indicator');
disp('       GPIO LED1                → TX indicator');
disp('  6. Generate Bitstream → export → launch Vitis/SDK');
disp('  7. ARM C code reads adsb_ip_top results via AXI-Lite registers,');
disp('     builds reply frame, and feeds it to AD9361 TX on 1090 MHz.');
disp(' ');
disp('  For the full transponder loop (RX decode + TX reply) running');
disp('  autonomously on the ZedBoard without MATLAB, the ARM PS handles:');
disp('    - DMA configuration for AD9361 RX capture');
disp('    - Triggering adsb_ip_top when a frame is ready');
disp('    - Reading decoded bits from AXI-Lite registers');
disp('    - Validating DF=B0 sync, extracting challenge');
disp('    - Building the 8D+ICAO+SQUAWK+PI reply frame');
disp('    - Feeding reply to AD9361 TX via DMA on 1090 MHz');
disp('    - Toggling LEDs via GPIO');
disp(' ');
disp('========================================================');
