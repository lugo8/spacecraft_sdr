// Testbench: reproduce best_power=0 with realistic noise-level inputs
//
// Uses LFSR-generated noise at realistic ADC levels (~±200 LSBs)
// to show that i_mix[13:10] truncation destroys the signal.

`timescale 1ns/1ps

module tb_fft_acq_noise;

    localparam N = 4096;

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

    wire [3:0]  dbg_state     = debug[25:22];
    wire [5:0]  dbg_epoch_cnt = debug[18:13];

    // LFSR for pseudo-random noise (16-bit Galois)
    reg [15:0] lfsr_i, lfsr_q;
    wire lfsr_fb_i = lfsr_i[0];
    wire lfsr_fb_q = lfsr_q[0];

    always @(posedge clk) begin
        lfsr_i <= {lfsr_fb_i, lfsr_i[15:1]} ^ (lfsr_fb_i ? 16'hB400 : 16'h0);
        lfsr_q <= {lfsr_fb_q, lfsr_q[15:1]} ^ (lfsr_fb_q ? 16'hB400 : 16'h0);
    end

    // Scale LFSR to realistic 12-bit ADC noise levels
    // GPS front-end with LNA=15, PGA=10: RMS ~100-300 LSBs in 12-bit
    // Use lfsr[15:7] as ~9-bit noise = ±256
    wire signed [11:0] noise_i = {{3{lfsr_i[15]}}, lfsr_i[15:7]};
    wire signed [11:0] noise_q = {{3{lfsr_q[15]}}, lfsr_q[15:7]};

    // Also observe the truncation directly
    wire signed [12:0] i_ext = {i_in[11], i_in};
    wire signed [13:0] i_mix_obs = {i_ext[12], i_ext} + 14'sd0; // simplified for freq=0
    wire signed [3:0]  i_trunc_current = i_mix_obs[13:10]; // current truncation
    wire signed [3:0]  i_trunc_fixed   = i_mix_obs[9:6];   // proposed fix

    integer cycle_count;
    integer nonzero_current, nonzero_fixed;

    initial begin
        $dumpfile("tb_fft_acq_noise.vcd");
        $dumpvars(0, tb_fft_acq_noise);

        rst = 1;
        start = 0;
        abort = 0;
        prn = 5'd1;
        carrier_freq = 32'd0;
        num_epochs = 6'd1;
        sample_valid = 0;
        i_in = 0;
        q_in = 0;
        lfsr_i = 16'hACE1;
        lfsr_q = 16'h1234;
        nonzero_current = 0;
        nonzero_fixed = 0;

        repeat (10) @(posedge clk);
        rst = 0;
        repeat (5) @(posedge clk);

        // Check truncation statistics on 4096 noise samples
        $display("=== Truncation analysis on 4096 noise samples ===");
        $display("  noise_i range: ~±256 (9-bit signed)");
        sample_valid = 1;
        repeat (4096) begin
            @(posedge clk);
            i_in = noise_i;
            q_in = noise_q;
            if (i_trunc_current != 0 && i_trunc_current != -1)
                nonzero_current = nonzero_current + 1;
            if (i_trunc_fixed != 0 && i_trunc_fixed != -1)
                nonzero_fixed = nonzero_fixed + 1;
        end
        $display("  i_mix[13:10] non-trivial values: %0d / 4096 (%0d%%)",
                 nonzero_current, nonzero_current * 100 / 4096);
        $display("  i_mix[9:6]   non-trivial values: %0d / 4096 (%0d%%)",
                 nonzero_fixed, nonzero_fixed * 100 / 4096);

        // Now run full acquisition with noise input
        $display("");
        $display("=== Running acquisition with noise input (current truncation) ===");

        @(posedge clk);
        start = 1;
        @(posedge clk);
        start = 0;

        cycle_count = 0;
        repeat (100000) begin
            @(posedge clk);
            cycle_count = cycle_count + 1;
            i_in = noise_i;
            q_in = noise_q;
            sample_valid = 1;

            if (done) begin
                $display("  Acquisition done at cycle %0d", cycle_count);
                $display("  best_phase=%0d  best_power=%0d", best_phase, best_power);
                if (best_power == 0)
                    $display("  >>> best_power=0 confirms truncation destroys signal <<<");
                else
                    $display("  >>> best_power>0 — signal survived truncation <<<");
                $finish;
            end
        end

        $display("  TIMEOUT: acquisition did not complete");
        $finish;
    end

endmodule
