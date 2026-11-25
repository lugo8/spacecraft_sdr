`timescale 1ns / 1ps

module tb_gps_correlator;

    localparam CLK_PERIOD = 12.9; // ~77.5 MHz
    localparam SAMPLE_DIV = 19;   // 77.5 MHz / 19 ≈ 4.08 MHz
    localparam CODE_FREQ_NOM = 32'h3FF00000;
    localparam CODE_LEN = 1023;

    // Register map
    localparam [2:0]
        REG_CTRL     = 3'd0,
        REG_CAR_FREQ = 3'd1,
        REG_COD_FREQ = 3'd2,
        REG_COD_PHASE= 3'd3, // write: code phase preset
        REG_IP       = 3'd3, // read: I-Prompt
        REG_QP       = 3'd4,
        REG_IE_QE    = 3'd5,
        REG_IL_QL    = 3'd6,
        REG_STATUS   = 3'd7;

    // Global channel addresses
    localparam [3:0]
        CH_GLOBAL = 4'hF,
        CH_GAIN   = 4'hC;

    // Global registers
    localparam [2:0]
        REG_GLOB_CTRL = 3'd0,
        REG_EPOCH_CNT = 3'd1,
        REG_VERSION   = 3'd2,
        REG_ACQ_CTRL  = 3'd3,
        REG_ACQ_CFG   = 3'd4,
        REG_ACQ_CAR   = 3'd5,
        REG_ACQ_POWER = 3'd6;

    reg clk = 0;
    reg rst = 1;
    always #(CLK_PERIOD/2) clk = ~clk;

    // I/Q signal (noise-like from LFSR for stages 0-2)
    reg signed [11:0] i_in, q_in;
    reg sample_valid;

    // Register interface
    reg  [6:0]  reg_addr;
    reg  [31:0] reg_wdata;
    wire [31:0] reg_rdata;
    reg         reg_wen;

    gps_correlator dut (
        .clk(clk), .rst(rst),
        .i_in(i_in), .q_in(q_in), .sample_valid(sample_valid),
        .reg_addr(reg_addr), .reg_wdata(reg_wdata),
        .reg_rdata(reg_rdata), .reg_wen(reg_wen)
    );

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

    // Simple LFSR for pseudo-random I/Q (noise floor)
    reg [15:0] lfsr = 16'hACE1;
    always @(posedge clk) begin
        lfsr <= {lfsr[14:0], lfsr[15] ^ lfsr[14] ^ lfsr[12] ^ lfsr[3]};
        if (sample_valid) begin
            i_in <= {{4{lfsr[11]}}, lfsr[11:4]} + 12'sd100;
            q_in <= {{4{lfsr[7]}},  lfsr[7:0]}  - 12'sd50;
        end
    end

    // Test infrastructure
    integer tests_run = 0, tests_pass = 0, tests_fail = 0;
    integer stage_pass;

    task gps_write(input [3:0] ch, input [2:0] regn, input [31:0] value);
        begin
            @(posedge clk); #1;
            reg_addr  = {ch, regn};
            reg_wdata = value;
            reg_wen   = 1;
            @(posedge clk); #1;
            reg_wen   = 0;
        end
    endtask

    function [31:0] gps_read_val;
        input [3:0] ch;
        input [2:0] regn;
        begin
            gps_read_val = 32'hx;
        end
    endfunction

    // Since functions can't have timing, use a task for read
    reg [31:0] rd_result;
    task gps_read(input [3:0] ch, input [2:0] regn);
        begin
            @(posedge clk); #1;
            reg_addr = {ch, regn};
            reg_wen  = 0;
            @(posedge clk); #1;
            rd_result = reg_rdata;
        end
    endtask

    task check(input [255:0] name, input cond);
        begin
            tests_run = tests_run + 1;
            if (cond) begin
                tests_pass = tests_pass + 1;
                $display("  PASS: %0s", name);
            end else begin
                tests_fail = tests_fail + 1;
                stage_pass = 0;
                $display("  FAIL: %0s", name);
            end
        end
    endtask

    // Wait N sample_valid pulses
    task wait_samples(input integer n);
        integer cnt;
        begin
            for (cnt = 0; cnt < n; cnt = cnt + 1)
                @(posedge sample_valid);
        end
    endtask

    // Wait N microseconds (approximate)
    task wait_us(input integer us);
        begin
            #(us * 1000);
        end
    endtask

    // ================================================================
    // Stage 0: SoC Alive — VERSION, EPOCH_CNT, CSR readback
    // ================================================================
    task test_stage0;
        reg [31:0] ver, e1, e2, rb;
        integer delta;
        begin
            $display("");
            $display("==== Stage 0: SoC alive ====");
            stage_pass = 1;

            // VERSION register
            gps_read(CH_GLOBAL, REG_VERSION);
            ver = rd_result;
            $display("  VERSION = 0x%08X", ver);
            check("VERSION == 0x47505307", ver == 32'h47505307);

            // Enable global + ch0 for epoch counting
            gps_write(CH_GLOBAL, REG_GLOB_CTRL, 1);
            gps_write(0, REG_COD_FREQ, CODE_FREQ_NOM);
            gps_write(0, REG_CTRL, (1 << 5) | 1); // enable ch0, PRN 1

            // Wait ~5 ms worth of samples for epochs to tick
            wait_samples(5 * 4096);

            gps_read(CH_GLOBAL, REG_EPOCH_CNT);
            e1 = rd_result;

            // Wait ~10 ms
            wait_samples(10 * 4096);

            gps_read(CH_GLOBAL, REG_EPOCH_CNT);
            e2 = rd_result;

            delta = e2 - e1;
            $display("  EPOCH_CNT: %0d -> %0d (delta=%0d in ~10 ms)", e1, e2, delta);
            check("EPOCH_CNT incrementing", delta > 0);
            check("EPOCH_CNT rate ~10 in 10 ms (5-15)", delta >= 5 && delta <= 15);

            // CSR write/readback
            gps_write(0, REG_CAR_FREQ, 32'hDEAD0042);
            gps_read(0, REG_CAR_FREQ);
            rb = rd_result;
            $display("  Write 0xDEAD0042, readback 0x%08X", rb);
            check("CSR write/readback", rb == 32'hDEAD0042);
            gps_write(0, REG_CAR_FREQ, 0);

            if (stage_pass)
                $display("  >> Stage 0: ALL PASS");
            else
                $display("  >> Stage 0: SOME FAILURES");
        end
    endtask

    // ================================================================
    // Stage 1: Carrier NCO + Mixer — accumulators respond
    // ================================================================
    task test_stage1;
        integer i, nonzero, changing;
        reg signed [31:0] ip, qp, prev_ip;
        begin
            $display("");
            $display("==== Stage 1: Carrier NCO + mixer ====");
            stage_pass = 1;

            // Disable all channels first
            for (i = 0; i < 8; i = i + 1)
                gps_write(i[3:0], REG_CTRL, 0);

            gps_write(CH_GLOBAL, REG_GLOB_CTRL, 1);
            gps_write(0, REG_CAR_FREQ, 0);
            gps_write(0, REG_COD_FREQ, CODE_FREQ_NOM);
            gps_write(0, REG_CTRL, (1 << 5) | 1);

            // Wait for a few epochs
            wait_samples(5 * 4096);

            // Read IP/QP several times
            $display("  Ch0 (PRN 1, carrier=0 Hz):");
            nonzero = 0;
            changing = 0;
            prev_ip = 0;
            for (i = 0; i < 10; i = i + 1) begin
                wait_samples(4096); // ~1 epoch
                gps_read(0, REG_IP);
                ip = $signed(rd_result);
                gps_read(0, REG_QP);
                qp = $signed(rd_result);
                $display("    sample %0d: IP=%0d QP=%0d", i, ip, qp);
                if (ip != 0 || qp != 0) nonzero = nonzero + 1;
                if (i > 0 && ip != prev_ip) changing = changing + 1;
                prev_ip = ip;
            end
            check("IP/QP non-zero (accumulator alive)", nonzero >= 5);
            check("IP changes between reads", changing >= 3);

            // Channel epoch counter advancing
            gps_read(0, REG_STATUS);
            begin : epoch_check
                reg [15:0] ec1, ec2;
                ec1 = rd_result[15:0];
                wait_samples(5 * 4096);
                gps_read(0, REG_STATUS);
                ec2 = rd_result[15:0];
                $display("  Ch0 epoch_count: %0d -> %0d", ec1, ec2);
                check("Channel epoch counter advancing", ec2 != ec1);
            end

            // Disable channels
            for (i = 0; i < 8; i = i + 1)
                gps_write(i[3:0], REG_CTRL, 0);

            if (stage_pass)
                $display("  >> Stage 1: ALL PASS");
            else
                $display("  >> Stage 1: SOME FAILURES");
        end
    endtask

    // ================================================================
    // Stage 2: Code Generator + Correlator (E/P/L arms, multi-channel)
    // ================================================================
    task test_stage2;
        integer i, el_nonzero;
        reg signed [31:0] ip, qp;
        reg [31:0] ie_qe, il_ql;
        reg signed [15:0] ie, qe, il, ql;
        integer ch0_active, ch1_active;
        begin
            $display("");
            $display("==== Stage 2: Code generator + correlator ====");
            stage_pass = 1;

            for (i = 0; i < 8; i = i + 1)
                gps_write(i[3:0], REG_CTRL, 0);

            gps_write(CH_GLOBAL, REG_GLOB_CTRL, 1);
            gps_write(0, REG_CAR_FREQ, 0);
            gps_write(0, REG_COD_FREQ, CODE_FREQ_NOM);
            gps_write(0, REG_CTRL, (1 << 5) | 1);

            wait_samples(10 * 4096);

            // E/P/L arms
            $display("  Ch0 E/P/L check (PRN 1):");
            el_nonzero = 0;
            for (i = 0; i < 5; i = i + 1) begin
                wait_samples(4096);
                gps_read(0, REG_IP); ip = $signed(rd_result);
                gps_read(0, REG_QP); qp = $signed(rd_result);
                gps_read(0, REG_IE_QE); ie_qe = rd_result;
                gps_read(0, REG_IL_QL); il_ql = rd_result;
                ie = $signed(ie_qe[31:16]);
                qe = $signed(ie_qe[15:0]);
                il = $signed(il_ql[31:16]);
                ql = $signed(il_ql[15:0]);
                $display("    IP=%6d QP=%6d  IE=%6d QE=%6d  IL=%6d QL=%6d",
                         ip, qp, ie, qe, il, ql);
                if (ie != 0 || qe != 0 || il != 0 || ql != 0) el_nonzero = el_nonzero + 1;
            end
            check("Early/Late arms non-zero", el_nonzero >= 3);

            // Multi-channel: ch0=PRN1, ch1=PRN15
            $display("");
            $display("  Multi-channel (ch0=PRN1, ch1=PRN15):");
            gps_write(0, REG_CAR_FREQ, 0);
            gps_write(0, REG_COD_FREQ, CODE_FREQ_NOM);
            gps_write(0, REG_CTRL, (1 << 5) | 1);
            gps_write(1, REG_CAR_FREQ, 0);
            gps_write(1, REG_COD_FREQ, CODE_FREQ_NOM);
            gps_write(1, REG_CTRL, (1 << 5) | 15);

            wait_samples(10 * 4096);

            ch0_active = 0;
            ch1_active = 0;
            for (i = 0; i < 5; i = i + 1) begin
                wait_samples(4096);
                gps_read(0, REG_IP);
                if ($signed(rd_result) != 0) ch0_active = ch0_active + 1;
                $display("    ch0 IP=%0d", $signed(rd_result));
                gps_read(1, REG_IP);
                if ($signed(rd_result) != 0) ch1_active = ch1_active + 1;
                $display("    ch1 IP=%0d", $signed(rd_result));
            end
            check("Ch0 (PRN 1) producing output", ch0_active >= 3);
            check("Ch1 (PRN 15) producing output", ch1_active >= 3);

            for (i = 0; i < 8; i = i + 1)
                gps_write(i[3:0], REG_CTRL, 0);

            if (stage_pass)
                $display("  >> Stage 2: ALL PASS");
            else
                $display("  >> Stage 2: SOME FAILURES");
        end
    endtask

    // ================================================================
    // Stage 3: FFT Acquisition Engine
    // ================================================================
    task test_stage3;
        integer i, completed;
        reg [31:0] status, power, mid_status, abort_status;
        reg [9:0] phase;
        reg [31:0] power_1ep, power_10ep;
        begin
            $display("");
            $display("==== Stage 3: FFT acquisition engine ====");
            stage_pass = 1;

            for (i = 0; i < 8; i = i + 1)
                gps_write(i[3:0], REG_CTRL, 0);
            gps_write(CH_GLOBAL, REG_GLOB_CTRL, 1);
            // Zero loop gains
            gps_write(CH_GAIN, 0, 0);
            gps_write(CH_GAIN, 1, 0);
            gps_write(CH_GAIN, 2, 0);
            gps_write(CH_GAIN, 3, 0);

            // Need a channel enabled to generate sample_valid epochs
            gps_write(0, REG_COD_FREQ, CODE_FREQ_NOM);
            gps_write(0, REG_CTRL, (1 << 5) | 1);

            // --- 3a: Engine completes ---
            $display("  3a: Acquisition engine completes");
            gps_write(CH_GLOBAL, REG_ACQ_CFG, (32'd1 << 16) | 6'd1); // PRN 1, 1 epoch
            gps_write(CH_GLOBAL, REG_ACQ_CAR, 32'd0);
            gps_write(CH_GLOBAL, REG_ACQ_CTRL, 32'd1); // start

            completed = 0;
            // 1 epoch acq: ~4096*19 capture + ~3*4096*2 FFT + 4096 scan ≈ 100K clocks
            for (i = 0; i < 2000; i = i + 1) begin
                wait_samples(100);
                gps_read(CH_GLOBAL, REG_ACQ_CTRL);
                status = rd_result;
                if (status[31]) begin
                    completed = 1;
                    gps_read(CH_GLOBAL, REG_ACQ_POWER);
                    power = rd_result;
                    phase = status[9:0];
                    $display("    Completed: power=%0d phase=%0d", power, phase);
                    i = 2000; // break
                end
            end
            check("Acquisition completes (not hung)", completed);

            // --- 3b: Active flag during acquisition ---
            $display("");
            $display("  3b: Active flag during acquisition");
            gps_write(CH_GLOBAL, REG_ACQ_CFG, (32'd1 << 16) | 6'd20); // 20 epochs
            gps_write(CH_GLOBAL, REG_ACQ_CAR, 32'd0);
            gps_write(CH_GLOBAL, REG_ACQ_CTRL, 32'd1);
            wait_samples(2000); // let it start
            gps_read(CH_GLOBAL, REG_ACQ_CTRL);
            mid_status = rd_result;
            $display("    Status during acq: 0x%08X (active=%0d)", mid_status, mid_status[30]);
            check("Active flag set during acquisition", mid_status[30]);

            // Wait for completion
            for (i = 0; i < 5000; i = i + 1) begin
                wait_samples(100);
                gps_read(CH_GLOBAL, REG_ACQ_CTRL);
                if (rd_result[31]) i = 5000;
            end

            // --- 3c: Abort works ---
            $display("");
            $display("  3c: Abort stops acquisition");
            gps_write(CH_GLOBAL, REG_ACQ_CFG, (32'd1 << 16) | 6'd40); // 40 epochs
            gps_write(CH_GLOBAL, REG_ACQ_CAR, 32'd0);
            gps_write(CH_GLOBAL, REG_ACQ_CTRL, 32'd1);
            wait_samples(5000); // let it start
            gps_write(CH_GLOBAL, REG_ACQ_CTRL, 32'd2); // abort
            wait_samples(100);
            gps_read(CH_GLOBAL, REG_ACQ_CTRL);
            abort_status = rd_result;
            $display("    Status after abort: 0x%08X", abort_status);
            check("Abort returns engine to idle", !abort_status[30] && !abort_status[31]);

            // --- 3d: Power scales with num_epochs ---
            $display("");
            $display("  3d: Power scales with num_epochs");

            // 1 epoch
            gps_write(CH_GLOBAL, REG_ACQ_CFG, (32'd1 << 16) | 6'd1);
            gps_write(CH_GLOBAL, REG_ACQ_CAR, 32'd0);
            gps_write(CH_GLOBAL, REG_ACQ_CTRL, 32'd1);
            power_1ep = 0;
            for (i = 0; i < 2000; i = i + 1) begin
                wait_samples(100);
                gps_read(CH_GLOBAL, REG_ACQ_CTRL);
                if (rd_result[31]) begin
                    gps_read(CH_GLOBAL, REG_ACQ_POWER);
                    power_1ep = rd_result;
                    i = 2000;
                end
            end

            // 10 epochs
            gps_write(CH_GLOBAL, REG_ACQ_CFG, (32'd1 << 16) | 6'd10);
            gps_write(CH_GLOBAL, REG_ACQ_CAR, 32'd0);
            gps_write(CH_GLOBAL, REG_ACQ_CTRL, 32'd1);
            power_10ep = 0;
            for (i = 0; i < 5000; i = i + 1) begin
                wait_samples(100);
                gps_read(CH_GLOBAL, REG_ACQ_CTRL);
                if (rd_result[31]) begin
                    gps_read(CH_GLOBAL, REG_ACQ_POWER);
                    power_10ep = rd_result;
                    i = 5000;
                end
            end

            $display("    1 epoch:  power=%0d", power_1ep);
            $display("    10 epoch: power=%0d", power_10ep);
            check("10-epoch power > 1-epoch power", power_10ep > power_1ep);

            for (i = 0; i < 8; i = i + 1)
                gps_write(i[3:0], REG_CTRL, 0);

            if (stage_pass)
                $display("  >> Stage 3: ALL PASS");
            else
                $display("  >> Stage 3: SOME FAILURES");
        end
    endtask

    // ================================================================
    // Main
    // ================================================================
    initial begin
        reg_addr  = 0;
        reg_wdata = 0;
        reg_wen   = 0;
        i_in      = 0;
        q_in      = 0;

        #(CLK_PERIOD * 20);
        @(posedge clk); #1;
        rst = 0;
        #(CLK_PERIOD * 10);

        test_stage0;
        test_stage1;
        test_stage2;
        test_stage3;

        $display("");
        $display("========================================");
        $display("  TOTAL: %0d/%0d passed, %0d failed",
                 tests_pass, tests_run, tests_fail);
        $display("========================================");

        if (tests_fail > 0)
            $display("  ** SOME TESTS FAILED **");
        else
            $display("  ** ALL TESTS PASSED **");

        $finish;
    end

    // Global timeout
    initial begin
        #(CLK_PERIOD * 50_000_000); // ~650 ms
        $display("");
        $display("*** GLOBAL TIMEOUT ***");
        $finish;
    end

endmodule
