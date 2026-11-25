`timescale 1ns / 1ps

module tb_gps_fft_acq;

    localparam CLK_PERIOD = 12.9; // ~77.5 MHz
    localparam N = 4096;
    localparam CODE_LEN = 1023;
    // Sample valid every 19 clocks: 77.5 MHz / 19 ≈ 4.08 MHz ≈ 4.096 MS/s
    localparam SAMPLE_DIV = 19;

    reg clk = 0;
    reg rst = 1;
    always #(CLK_PERIOD/2) clk = ~clk;

    reg signed [11:0] i_in, q_in;
    reg sample_valid;
    reg start, abort;
    reg [4:0] prn;
    reg [31:0] carrier_freq;
    reg [5:0] num_epochs;

    wire done, active;
    wire [9:0] best_phase;
    wire [31:0] best_power;

    gps_fft_acq dut (
        .clk(clk), .rst(rst),
        .i_in(i_in), .q_in(q_in), .sample_valid(sample_valid),
        .start(start), .abort(abort),
        .prn(prn), .carrier_freq(carrier_freq), .num_epochs(num_epochs),
        .done(done), .active(active),
        .best_phase(best_phase), .best_power(best_power)
    );

    // Generate reference C/A code for PRN 1 using a second ca_gen instance
    reg ref_rst, ref_chip_en;
    wire ref_code_out, ref_epoch, ref_busy;

    gps_ca_gen ref_ca (
        .clk(clk), .rst(ref_rst),
        .prn(5'd1),
        .chip_en(ref_chip_en),
        .phase_wen(1'b0), .phase_in(10'd0),
        .code_out(ref_code_out),
        .epoch(ref_epoch),
        .busy(ref_busy)
    );

    // Store the full 1023-chip C/A code for PRN 1
    reg ref_code_array [0:CODE_LEN-1];
    integer gen_i;

    // NCO for generating test samples at desired code phase offset
    localparam real FS = 4096000.0;
    localparam real FCODE = 1023000.0;
    localparam integer CODE_PHASE_OFFSET = 200; // chips — this is what we want to detect

    // Sample generation state
    reg [31:0] code_nco;
    localparam [31:0] CODE_NCO_STEP = 32'd1073217536; // 1.023e6/4.096e6 * 2^32

    // Monitor FSM state via hierarchical reference
    wire [3:0] fsm_state = dut.state;
    wire [5:0] fsm_epoch = dut.epoch_cnt;
    wire [12:0] fsm_cnt  = dut.cnt;
    wire fsm_fft_sync    = dut.fft_sync;
    wire fsm_out_sync    = dut.out_sync_seen;
    wire fsm_out_active  = dut.out_active;
    wire fsm_fft_ce      = dut.fft_ce;

    reg [3:0] prev_state;

    // State name strings for display
    function [79:0] state_name;
        input [3:0] s;
        case (s)
            4'd0: state_name = "IDLE      ";
            4'd1: state_name = "GEN_CODE  ";
            4'd2: state_name = "CLR_PWR   ";
            4'd3: state_name = "CAPTURE   ";
            4'd4: state_name = "FFT_SIG   ";
            4'd5: state_name = "FFT_MUL   ";
            4'd6: state_name = "FFT_IFFT  ";
            4'd7: state_name = "NEXT_EPOCH";
            4'd8: state_name = "FIND_BEST ";
            4'd9: state_name = "DONE      ";
            default: state_name = "???       ";
        endcase
    endfunction

    // Monitor state transitions
    always @(posedge clk) begin
        prev_state <= fsm_state;
        if (fsm_state != prev_state)
            $display("[%0t] STATE: %0s -> %0s  epoch=%0d cnt=%0d",
                     $time, state_name(prev_state), state_name(fsm_state),
                     fsm_epoch, fsm_cnt);
    end

    // Monitor FFT outputs and pipeline values
    wire signed [10:0] dbg_fft_or = dut.fft_or;
    wire signed [10:0] dbg_fft_oi = dut.fft_oi;
    wire [10:0] dbg_abs_or = dut.abs_or;
    wire [10:0] dbg_abs_oi = dut.abs_oi;
    wire [11:0] dbg_mag = dut.mag;
    wire dbg_pipe_acc_valid = dut.pipe_acc_valid;
    wire [11:0] dbg_pipe_acc_addr = dut.pipe_acc_addr;
    wire [11:0] dbg_pipe_mag = dut.pipe_mag;
    wire [15:0] dbg_pwr_wd = dut.pwr_wd;
    wire dbg_pwr_we = dut.pwr_we;
    wire [11:0] dbg_out_cnt = dut.out_cnt;

    // Count FFT sync pulses per state
    reg [31:0] sync_count;
    always @(posedge clk) begin
        if (fsm_state != prev_state) sync_count <= 0;
        else if (fsm_fft_ce && fsm_fft_sync) sync_count <= sync_count + 1;
    end

    // Print first few FFT outputs in each pass
    reg [31:0] out_sample_cnt;
    always @(posedge clk) begin
        if (fsm_state != prev_state) out_sample_cnt <= 0;
        else if (fsm_out_active && fsm_fft_ce) begin
            out_sample_cnt <= out_sample_cnt + 1;
            if (out_sample_cnt < 5 || (out_sample_cnt % 1000 == 0))
                $display("[%0t]   %0s out[%0d]: fft_or=%0d fft_oi=%0d mag=%0d",
                         $time, state_name(fsm_state), dbg_out_cnt,
                         dbg_fft_or, dbg_fft_oi, dbg_mag);
        end
    end

    // Monitor accumulation writes
    reg [31:0] acc_write_cnt;
    always @(posedge clk) begin
        if (fsm_state != prev_state) acc_write_cnt <= 0;
        else if (dbg_pipe_acc_valid && fsm_state == 4'd6) begin
            acc_write_cnt <= acc_write_cnt + 1;
            if (acc_write_cnt < 5 || (acc_write_cnt % 1000 == 0))
                $display("[%0t]   ACC_WR[%0d]: addr=%0d mag=%0d pwr_wd=%0d",
                         $time, acc_write_cnt, dbg_pipe_acc_addr, dbg_pipe_mag, dbg_pwr_wd);
        end
    end

    // Sample valid generator
    reg [4:0] sample_div_cnt;
    always @(posedge clk) begin
        if (rst) begin
            sample_div_cnt <= 0;
            sample_valid <= 0;
        end else begin
            if (sample_div_cnt == SAMPLE_DIV - 1) begin
                sample_div_cnt <= 0;
                sample_valid <= 1;
            end else begin
                sample_div_cnt <= sample_div_cnt + 1;
                sample_valid <= 0;
            end
        end
    end

    // Sample generation: C/A code at CODE_PHASE_OFFSET, zero Doppler
    reg [9:0] sample_chip_idx;
    always @(posedge clk) begin
        if (rst) begin
            code_nco <= 0;
            i_in <= 0;
            q_in <= 0;
        end else if (sample_valid) begin
            // Advance code NCO
            code_nco <= code_nco + CODE_NCO_STEP;
            // Chip index with offset
            sample_chip_idx = ((code_nco + CODE_NCO_STEP) >> 22) % CODE_LEN;
            sample_chip_idx = (sample_chip_idx + CODE_PHASE_OFFSET) % CODE_LEN;
            // I = +/-1 (12-bit), Q = 0, zero Doppler
            if (ref_code_array[sample_chip_idx])
                i_in <= 12'sd2000;
            else
                i_in <= -12'sd2000;
            q_in <= 12'd0;
        end
    end

    // Check twiddle factor ROM and sig_buf
    wire [15:0] dbg_sig_rd = dut.sig_rd;
    wire [7:0]  dbg_fft_in = dut.fft_in;
    wire signed [3:0] dbg_i_trunc = dut.i_trunc;
    wire signed [3:0] dbg_q_trunc = dut.q_trunc;
    wire signed [13:0] dbg_i_mix = dut.i_mix;

    // Print sig_buf writes during CAPTURE
    reg [31:0] cap_cnt;
    always @(posedge clk) begin
        if (fsm_state == 4'd3 && dut.sig_we) begin
            cap_cnt <= cap_cnt + 1;
            if (cap_cnt < 5 || cap_cnt == 100 || cap_cnt == 1000)
                $display("[%0t] CAPTURE[%0d]: i_in=%0d i_mix=%0d i_trunc=%0d sig_wd=0x%04X",
                         $time, cap_cnt, $signed(dut.i_in), dbg_i_mix, dbg_i_trunc, dut.sig_wd);
        end
        if (fsm_state != prev_state && fsm_state == 4'd3)
            cap_cnt <= 0;
    end

    // Print sig_buf reads and FFT input at start of FFT_SIG
    reg [31:0] fft_in_cnt;
    always @(posedge clk) begin
        if (fsm_state == 4'd4 && dut.fft_ce) begin
            fft_in_cnt <= fft_in_cnt + 1;
            if (fft_in_cnt < 5)
                $display("[%0t] FFT_SIG_IN[%0d]: sig_rd=0x%04X fft_in=0x%02X fft_rst=%0b",
                         $time, fft_in_cnt, dbg_sig_rd, dbg_fft_in, dut.fft_rst_r);
        end
        if (fsm_state != prev_state && fsm_state == 4'd4)
            fft_in_cnt <= 0;
    end

    // Skip VCD for faster sim
    //initial begin
    //    $dumpfile("tb_gps_fft_acq.vcd");
    //    $dumpvars(0, tb_gps_fft_acq);
    //end

    initial begin
        // Initialize
        start = 0;
        abort = 0;
        prn = 5'd1;
        carrier_freq = 32'd0; // zero Doppler
        num_epochs = 6'd1;    // single epoch for fast test
        i_in = 0;
        q_in = 0;
        ref_rst = 1;
        ref_chip_en = 0;

        // Reset
        #(CLK_PERIOD * 10);
        @(posedge clk);
        #1;
        rst = 0;
        ref_rst = 0;
        #(CLK_PERIOD * 5);

        // Generate reference C/A code for PRN 1
        $display("Generating reference PRN 1 C/A code...");
        for (gen_i = 0; gen_i < CODE_LEN; gen_i = gen_i + 1) begin
            @(posedge clk);
            ref_chip_en = 1;
            @(posedge clk);
            ref_chip_en = 0;
            // Wait for ca_gen to settle
            @(posedge clk);
            ref_code_array[gen_i] = ref_code_out;
        end
        $display("Reference code generated. First 10 chips: %b%b%b%b%b%b%b%b%b%b",
                 ref_code_array[0], ref_code_array[1], ref_code_array[2],
                 ref_code_array[3], ref_code_array[4], ref_code_array[5],
                 ref_code_array[6], ref_code_array[7], ref_code_array[8],
                 ref_code_array[9]);

        // Start acquisition
        $display("");
        $display("=== Starting FFT Acquisition ===");
        $display("PRN=1, carrier_freq=0 (zero Doppler), num_epochs=1");
        $display("Expected code phase offset: %0d chips", CODE_PHASE_OFFSET);
        $display("");

        @(posedge clk);
        #1;
        start = 1;
        @(posedge clk);
        #1;
        start = 0;

        // Wait for completion or timeout
        // Worst case: 1023 gen + 4096 clr + 4096*19 capture + 3*~9000 fft + 4096 scan
        // ≈ 120000 clocks
        fork
            begin
                wait(done);
                $display("");
                $display("=== ACQUISITION COMPLETE ===");
                $display("best_phase = %0d (expected ~%0d)", best_phase, CODE_PHASE_OFFSET);
                $display("best_power = %0d (0x%08X)", best_power, best_power);
                $display("");
                if (best_power == 0)
                    $display("*** FAIL: best_power is zero — FFT likely not producing output ***");
                else if (best_phase == CODE_PHASE_OFFSET ||
                         best_phase == CODE_PHASE_OFFSET - 1 ||
                         best_phase == CODE_PHASE_OFFSET + 1)
                    $display("*** PASS: detected code phase matches expected ***");
                else
                    $display("*** FAIL: phase mismatch (got %0d, expected ~%0d) ***",
                             best_phase, CODE_PHASE_OFFSET);
                #(CLK_PERIOD * 10);
                $finish;
            end
            begin
                #(CLK_PERIOD * 500000);
                $display("");
                $display("*** TIMEOUT after 500000 clocks ***");
                $display("FSM stuck in state: %0s", state_name(fsm_state));
                $display("epoch=%0d cnt=%0d fft_sync=%0b out_sync=%0b out_active=%0b fft_ce=%0b",
                         fsm_epoch, fsm_cnt, fsm_fft_sync, fsm_out_sync,
                         fsm_out_active, fsm_fft_ce);
                $finish;
            end
        join
    end

endmodule
