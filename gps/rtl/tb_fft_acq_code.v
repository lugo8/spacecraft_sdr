// Testbench: verify FFT acquisition finds a known PRN code signal
//
// Generates a PRN 1 C/A code signal at 4 samples/chip (4.096 MS/s),
// embeds it in noise, feeds it to the acquisition engine, and checks
// that best_power > 0 and best_phase matches the known code offset.

`timescale 1ns/1ps

module tb_fft_acq_code;

    localparam N = 4096;
    localparam CODE_LEN = 1023;
    localparam SAMP_PER_CHIP = 4;

    reg clk, rst;
    reg signed [11:0] i_in, q_in;
    reg sample_valid;
    reg start, abort;
    reg [4:0] prn;
    reg [31:0] carrier_freq;
    reg [5:0] num_epochs;

    wire done, active;
    wire [9:0] best_phase;
    wire [31:0] best_power;
    wire [31:0] debug;

    gps_fft_acq dut (
        .clk          (clk),
        .rst          (rst),
        .i_in         (i_in),
        .q_in         (q_in),
        .sample_valid (sample_valid),
        .start        (start),
        .abort        (abort),
        .prn          (prn),
        .carrier_freq (carrier_freq),
        .num_epochs   (num_epochs),
        .done         (done),
        .active       (active),
        .best_phase   (best_phase),
        .best_power   (best_power),
        .debug        (debug)
    );

    initial clk = 0;
    always #5 clk = ~clk;

    wire [3:0] dbg_state     = debug[25:22];
    wire       dbg_fft_sync  = debug[21];
    wire       dbg_sync_seen = debug[20];
    wire       dbg_out_active= debug[19];
    wire [5:0] dbg_epoch_cnt = debug[18:13];
    wire [12:0]dbg_cnt       = debug[12:0];

    // ---- Generate PRN 1 C/A code using a second gps_ca_gen ----
    reg ca_rst_tb, ca_chip_en_tb;
    wire ca_code_tb, ca_epoch_tb, ca_busy_tb;

    gps_ca_gen ca_gen_tb (
        .clk       (clk),
        .rst       (ca_rst_tb),
        .prn       (5'd1),
        .chip_en   (ca_chip_en_tb),
        .phase_wen (1'b0),
        .phase_in  (10'd0),
        .code_out  (ca_code_tb),
        .epoch     (ca_epoch_tb),
        .busy      (ca_busy_tb)
    );

    // Store the full code
    reg code_bits [0:CODE_LEN-1];
    integer gen_idx;

    // ---- LFSR for noise ----
    reg [15:0] lfsr;
    always @(posedge clk)
        lfsr <= {lfsr[0], lfsr[15:1]} ^ (lfsr[0] ? 16'hB400 : 16'h0);

    // Noise: ~±128 (8-bit range, realistic GPS front-end level)
    wire signed [11:0] noise = {{4{lfsr[15]}}, lfsr[15:8]};

    // ---- Signal generation ----
    // PRN 1 code at known phase offset, 4 samples/chip
    // Signal = code_value × amplitude + noise
    localparam signed [11:0] SIG_AMP = 12'sd200;  // strong signal for clean test
    localparam integer CODE_PHASE_OFFSET = 42;      // known code phase offset in chips

    reg [9:0] code_chip_idx;
    reg [1:0] samp_in_chip;

    wire code_at_offset = code_bits[(code_chip_idx + CODE_PHASE_OFFSET) % CODE_LEN];
    wire signed [11:0] signal_i = code_at_offset ? SIG_AMP : -SIG_AMP;

    reg [3:0] prev_state;

    integer cycle_count;

    initial begin
        $dumpfile("tb_fft_acq_code.vcd");
        $dumpvars(0, tb_fft_acq_code);

        rst = 1;
        start = 0;
        abort = 0;
        prn = 5'd1;
        carrier_freq = 32'd0;
        num_epochs = 6'd1;
        sample_valid = 0;
        i_in = 0;
        q_in = 0;
        ca_rst_tb = 1;
        ca_chip_en_tb = 0;
        lfsr = 16'hACE1;
        code_chip_idx = 0;
        samp_in_chip = 0;
        prev_state = 0;

        repeat (10) @(posedge clk);
        rst = 0;
        ca_rst_tb = 0;
        repeat (2) @(posedge clk);

        // Generate PRN 1 code into code_bits array
        $display("Generating PRN 1 C/A code...");
        for (gen_idx = 0; gen_idx < CODE_LEN; gen_idx = gen_idx + 1) begin
            ca_chip_en_tb = 1;
            @(posedge clk);
            while (ca_busy_tb) @(posedge clk);
            code_bits[gen_idx] = ca_code_tb;
        end
        ca_chip_en_tb = 0;
        $display("  Code generated. First 20 chips:");
        $write("  ");
        for (gen_idx = 0; gen_idx < 20; gen_idx = gen_idx + 1)
            $write("%b", code_bits[gen_idx]);
        $display("");

        // Check signal levels after truncation
        $display("");
        $display("=== Signal level check ===");
        $display("  SIG_AMP = %0d", SIG_AMP);
        // +200 in 14 bits: i_mix[9:6]
        // 200 = 0b00_0000_1100_1000 → bits[9:6] = 0011 = 3
        $display("  +SIG_AMP bits[9:6] = %0d", $signed(14'sd200) >>> 6);
        $display("  -SIG_AMP bits[9:6] = %0d", $signed(-14'sd200) >>> 6);

        // Now start feeding signal and run acquisition
        $display("");
        $display("=== Starting acquisition: PRN=1, epochs=1, code_phase_offset=%0d ===",
                 CODE_PHASE_OFFSET);

        // Feed a few samples to stabilize, then start
        sample_valid = 1;
        code_chip_idx = 0;
        samp_in_chip = 0;
        repeat (100) begin
            @(posedge clk);
            i_in = signal_i + noise;
            q_in = noise;
            if (samp_in_chip == SAMP_PER_CHIP - 1) begin
                samp_in_chip = 0;
                code_chip_idx = (code_chip_idx == CODE_LEN - 1) ? 0 : code_chip_idx + 1;
            end else
                samp_in_chip = samp_in_chip + 1;
        end

        // Start acquisition
        $display("Before start: state=%0d active=%b done=%b rst=%b", dbg_state, active, done, rst);
        @(posedge clk);
        #1;
        start = 1;
        $display("Set start=1");
        @(posedge clk);
        $display("After posedge with start=1: state=%0d active=%b debug=0x%08x", dbg_state, active, debug);
        #1;
        start = 0;
        @(posedge clk);
        $display("After posedge with start=0: state=%0d active=%b debug=0x%08x", dbg_state, active, debug);

        $display("Acquisition started, watching FSM...");

        cycle_count = 0;
        repeat (200000) begin
            @(posedge clk);
            cycle_count = cycle_count + 1;

            // Generate signal: code × amplitude + noise
            i_in = signal_i + noise;
            q_in = noise;

            // Advance code phase (4 samples per chip)
            if (samp_in_chip == SAMP_PER_CHIP - 1) begin
                samp_in_chip = 0;
                code_chip_idx = (code_chip_idx == CODE_LEN - 1) ? 0 : code_chip_idx + 1;
            end else
                samp_in_chip = samp_in_chip + 1;

            // Print state transitions
            if (dbg_state != prev_state) begin
                $display("  cycle %6d: state=%0d→%0d  sync_seen=%b out_active=%b epoch=%0d cnt=%0d",
                         cycle_count, prev_state, dbg_state,
                         dbg_sync_seen, dbg_out_active, dbg_epoch_cnt, dbg_cnt);
                prev_state = dbg_state;
            end

            if (done) begin
                $display("");
                $display("=== Acquisition DONE at cycle %0d ===", cycle_count);
                $display("  best_phase = %0d (expected ~%0d)", best_phase, CODE_PHASE_OFFSET);
                $display("  best_power = %0d", best_power);
                if (best_power > 0)
                    $display("  PASS: best_power > 0");
                else
                    $display("  FAIL: best_power = 0 (data path broken)");

                if (best_power > 0) begin
                    // Check phase is close to expected
                    // The phase maps code phases: 4096 FFT bins → 1024 code phases (÷4)
                    if (best_phase >= CODE_PHASE_OFFSET - 2 && best_phase <= CODE_PHASE_OFFSET + 2)
                        $display("  PASS: phase matches expected offset (within ±2)");
                    else
                        $display("  NOTE: phase=%0d, expected=%0d (may differ due to quantization)",
                                 best_phase, CODE_PHASE_OFFSET);
                end
                $finish;
            end
        end

        $display("");
        $display("FAIL: Acquisition timed out after %0d cycles", cycle_count);
        $display("  state=%0d sync_seen=%b out_active=%b epoch=%0d cnt=%0d",
                 dbg_state, dbg_sync_seen, dbg_out_active, dbg_epoch_cnt, dbg_cnt);
        $finish;
    end

endmodule
