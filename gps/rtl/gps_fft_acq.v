// FFT-based GPS L1 C/A acquisition engine — drop-in replacement for gps_acq_engine
//
// Uses a 4096-point pipelined FFT (dblclockfft, 4-bit input) for circular
// correlation: all 4096 code phases tested simultaneously per Doppler bin.
//
// Per-epoch flow (3 FFT passes, single core reused):
//   Pass 1 (S_FFT_SIG):  sig_buf → FFT → truncate → sig_buf (overwrite)
//   Pass 2 (S_FFT_MUL):  code_rom → FFT → multiply S[k]×conj(C[k]) → conj → sig_buf
//   Pass 3 (S_FFT_IFFT): sig_buf (bit-rev read) → FFT → |I|+|Q| → pwr_ram
//
// IFFT uses the conjugate trick: IFFT(P) = conj(FFT(conj(P)))/N.
// conj(P) stored in pass 2; magnitude is conjugation-invariant.
//
// Timing at 77.5 MHz system clock:
//   Per FFT pass: 2×4096 = 8192 cycles ≈ 106 µs
//   Per epoch: 1 ms (capture) + 3×106 µs (compute) ≈ 1.32 ms
//   20-epoch trial: ~26 ms (vs ~1300 ms time-domain engine)
//
// Resources (ECP5):
//   FFT core (dblclockfft 4-bit):  17 DP16KD, ~1900 LUT, 24 MULT18
//   sig_buf (4096×16):              4 DP16KD
//   pwr_ram (4096×16):              4 DP16KD
//   Total:                         25 DP16KD, ~3K LUT, 28 MULT18

module gps_fft_acq (
    input  wire        clk,
    input  wire        rst,

    input  wire signed [11:0] i_in,
    input  wire signed [11:0] q_in,
    input  wire        sample_valid,

    input  wire        start,
    input  wire        abort,
    input  wire [4:0]  prn,
    input  wire [31:0] carrier_freq,
    input  wire [5:0]  num_epochs,

    output wire        done,
    output wire        active,
    output reg  [9:0]  best_phase,
    output reg  [31:0] best_power
);

    localparam N         = 4096;
    localparam LGN       = 12;
    localparam CODE_LEN  = 1023;
    localparam IWIDTH    = 4;
    localparam OWIDTH    = 11;
    localparam BW        = 8;

    // ---- States ----
    localparam [3:0]
        S_IDLE       = 4'd0,
        S_GEN_CODE   = 4'd1,
        S_CLR_PWR    = 4'd2,
        S_CAPTURE    = 4'd3,
        S_FFT_SIG    = 4'd4,
        S_FFT_MUL    = 4'd5,
        S_FFT_IFFT   = 4'd6,
        S_NEXT_EPOCH = 4'd7,
        S_FIND_BEST  = 4'd8,
        S_DONE       = 4'd9;

    reg [3:0] state;
    assign done   = (state == S_DONE);
    assign active = (state != S_IDLE) && (state != S_DONE);

    // ---- Carrier NCO ----
    reg [31:0] car_phase;
    wire cos_neg = car_phase[31] ^ car_phase[30];
    wire sin_neg = car_phase[31];
    wire signed [12:0] i_ext = {i_in[11], i_in};
    wire signed [12:0] q_ext = {q_in[11], q_in};
    wire signed [12:0] i_cos = cos_neg ? -i_ext : i_ext;
    wire signed [12:0] q_sin = sin_neg ? -q_ext : q_ext;
    wire signed [12:0] q_cos = cos_neg ? -q_ext : q_ext;
    wire signed [12:0] i_sin = sin_neg ? -i_ext : i_ext;
    wire signed [13:0] i_mix = i_cos + q_sin;
    wire signed [13:0] q_mix = q_cos - i_sin;
    wire signed [IWIDTH-1:0] i_trunc = i_mix[13:10];
    wire signed [IWIDTH-1:0] q_trunc = q_mix[13:10];

    // ---- C/A code generator ----
    wire ca_rst = rst | (state == S_IDLE);
    reg  ca_chip_en;
    wire ca_code_out, ca_epoch_out, ca_busy;

    gps_ca_gen ca_gen (
        .clk       (clk),
        .rst       (ca_rst),
        .prn       (prn),
        .chip_en   (ca_chip_en),
        .phase_wen (1'b0),
        .phase_in  (10'd0),
        .code_out  (ca_code_out),
        .epoch     (ca_epoch_out),
        .busy      (ca_busy)
    );

    reg [3:0] code_rom [0:CODE_LEN-1];
    reg [9:0] gen_ptr;

    // ---- sig_buf: N × 2*BW dual-port block RAM ----
    reg [2*BW-1:0] sig_mem [0:N-1];
    reg [LGN-1:0]  sig_wa, sig_ra;
    reg [2*BW-1:0] sig_wd;
    reg             sig_we;
    reg [2*BW-1:0] sig_rd;

    always @(posedge clk) begin
        sig_rd <= sig_mem[sig_ra];
        if (sig_we) sig_mem[sig_wa] <= sig_wd;
    end

    // ---- pwr_ram: N × 16-bit dual-port block RAM ----
    reg [15:0] pwr_mem [0:N-1];
    reg [LGN-1:0] pwr_wa, pwr_ra;
    reg [15:0]     pwr_wd;
    reg            pwr_we;
    reg [15:0]     pwr_rd;

    always @(posedge clk) begin
        pwr_rd <= pwr_mem[pwr_ra];
        if (pwr_we) pwr_mem[pwr_wa] <= pwr_wd;
    end

    // ---- FFT instance ----
    reg  fft_rst_r, fft_ce;
    reg  [2*IWIDTH-1:0] fft_in;
    wire [2*OWIDTH-1:0] fft_out;
    wire fft_sync;

    fftmain fft_inst (
        .i_clk    (clk),
        .i_reset  (fft_rst_r),
        .i_ce     (fft_ce),
        .i_sample (fft_in),
        .o_result (fft_out),
        .o_sync   (fft_sync)
    );

    wire signed [OWIDTH-1:0] fft_or = fft_out[2*OWIDTH-1:OWIDTH];
    wire signed [OWIDTH-1:0] fft_oi = fft_out[OWIDTH-1:0];

    // ---- Bit-reversal function ----
    function [LGN-1:0] bitrev;
        input [LGN-1:0] a;
        integer b;
        begin
            for (b = 0; b < LGN; b = b + 1)
                bitrev[b] = a[LGN-1-b];
        end
    endfunction

    // ---- Complex multiply: S × conj(C) ----
    reg signed [OWIDTH-1:0] pipe_cr, pipe_ci;

    wire signed [BW-1:0] mul_sr = sig_rd[2*BW-1:BW];
    wire signed [BW-1:0] mul_si = sig_rd[BW-1:0];

    wire signed [BW+OWIDTH-1:0] m_rr = mul_sr * pipe_cr;
    wire signed [BW+OWIDTH-1:0] m_ii = mul_si * pipe_ci;
    wire signed [BW+OWIDTH-1:0] m_ir = mul_si * pipe_cr;
    wire signed [BW+OWIDTH-1:0] m_ri = mul_sr * pipe_ci;

    wire signed [BW+OWIDTH:0] prod_r = {m_rr[BW+OWIDTH-1], m_rr} + {m_ii[BW+OWIDTH-1], m_ii};
    wire signed [BW+OWIDTH:0] prod_i = {m_ir[BW+OWIDTH-1], m_ir} - {m_ri[BW+OWIDTH-1], m_ri};
    wire signed [BW+OWIDTH:0] conj_i = -prod_i;

    localparam PW = BW + OWIDTH;
    wire signed [BW-1:0] store_r = prod_r[PW:PW-BW+1];
    wire signed [BW-1:0] store_i = conj_i[PW:PW-BW+1];

    // ---- Magnitude for IFFT accumulation ----
    wire [OWIDTH-1:0] abs_or = fft_or[OWIDTH-1] ? (~fft_or + 1'b1) : fft_or;
    wire [OWIDTH-1:0] abs_oi = fft_oi[OWIDTH-1] ? (~fft_oi + 1'b1) : fft_oi;
    wire [OWIDTH:0]   mag    = {1'b0, abs_or} + {1'b0, abs_oi};

    // ---- Counters and pipeline registers ----
    reg [LGN:0]   cnt;
    reg [LGN-1:0] out_cnt;
    reg            out_active;
    reg            out_sync_seen;
    reg [5:0]      epoch_cnt;

    reg [LGN-1:0]  pipe_mul_addr;
    reg             pipe_mul_valid;

    reg [OWIDTH:0]  pipe_mag;
    reg [LGN-1:0]   pipe_acc_addr;
    reg              pipe_acc_valid;

    reg [LGN-1:0] scan_addr;
    reg [1:0]     scan_pipe;

    // ---- Single always block: state machine + FFT output tracking ----
    always @(posedge clk) begin
        // Default strobes off
        sig_we         <= 1'b0;
        pwr_we         <= 1'b0;
        fft_rst_r      <= 1'b0;
        fft_ce         <= 1'b0;
        fft_in         <= {2*IWIDTH{1'b0}};
        ca_chip_en     <= 1'b0;
        pipe_mul_valid <= 1'b0;
        pipe_acc_valid <= 1'b0;

        // Multiply pipeline writeback
        if (pipe_mul_valid) begin
            sig_we <= 1'b1;
            sig_wa <= pipe_mul_addr;
            sig_wd <= {store_r, store_i};
        end

        // Accumulation pipeline writeback
        if (pipe_acc_valid) begin
            pwr_we <= 1'b1;
            pwr_wa <= pipe_acc_addr;
            pwr_wd <= pwr_rd + {{(16-OWIDTH-1){1'b0}}, pipe_mag};
        end

        if (rst) begin
            state         <= S_IDLE;
            car_phase     <= 32'd0;
            gen_ptr       <= 10'd0;
            cnt           <= 0;
            epoch_cnt     <= 6'd0;
            out_active    <= 1'b0;
            out_sync_seen <= 1'b0;
            out_cnt       <= 0;
            best_phase    <= 10'd0;
            best_power    <= 32'd0;
            pipe_cr       <= 0;
            pipe_ci       <= 0;
        end else if (abort && state != S_IDLE) begin
            state <= S_IDLE;
        end else begin

            // ---- FFT output tracking (runs during FFT states) ----
            if (fft_ce) begin
                if (fft_sync && !out_sync_seen) begin
                    out_sync_seen <= 1'b1;
                    out_active    <= 1'b1;
                    out_cnt       <= 0;
                end else if (out_active) begin
                    if (out_cnt == N[LGN-1:0] - 1)
                        out_active <= 1'b0;
                    else
                        out_cnt <= out_cnt + 1;
                end
            end

            case (state)

            // ============================================================
            S_IDLE: begin
                if (start) begin
                    gen_ptr   <= 10'd0;
                    epoch_cnt <= 6'd0;
                    state     <= S_GEN_CODE;
                end
            end

            // ============================================================
            S_GEN_CODE: begin
                ca_chip_en <= (gen_ptr < CODE_LEN - 1);
                if (!ca_busy || gen_ptr == 0) begin
                    code_rom[gen_ptr] <= {3'b0, ca_code_out};
                    gen_ptr <= gen_ptr + 10'd1;
                    if (gen_ptr == CODE_LEN - 1) begin
                        cnt   <= 0;
                        state <= S_CLR_PWR;
                    end
                end
            end

            // ============================================================
            S_CLR_PWR: begin
                pwr_we <= 1'b1;
                pwr_wa <= cnt[LGN-1:0];
                pwr_wd <= 16'd0;
                cnt    <= cnt + 1;
                if (cnt[LGN-1:0] == N[LGN-1:0] - 1) begin
                    car_phase <= 32'd0;
                    cnt       <= 0;
                    state     <= S_CAPTURE;
                end
            end

            // ============================================================
            S_CAPTURE: begin
                if (sample_valid) begin
                    sig_we <= 1'b1;
                    sig_wa <= cnt[LGN-1:0];
                    sig_wd <= {{(BW-IWIDTH){i_trunc[IWIDTH-1]}}, i_trunc,
                               {(BW-IWIDTH){q_trunc[IWIDTH-1]}}, q_trunc};
                    car_phase <= car_phase + carrier_freq;
                    cnt <= cnt + 1;
                    if (cnt[LGN-1:0] == N[LGN-1:0] - 1) begin
                        cnt           <= 0;
                        fft_rst_r     <= 1'b1;
                        out_sync_seen <= 1'b0;
                        out_active    <= 1'b0;
                        sig_ra        <= 0;
                        state         <= S_FFT_SIG;
                    end
                end
            end

            // ============================================================
            // Pass 1: FFT signal from sig_buf → store output back
            S_FFT_SIG: begin
                fft_ce <= 1'b1;

                if (cnt < N) begin
                    fft_in <= {sig_rd[BW+IWIDTH-1:BW], sig_rd[IWIDTH-1:0]};
                    if (cnt < N - 1)
                        sig_ra <= cnt[LGN-1:0] + 1;
                end

                cnt <= cnt + 1;

                if (out_active) begin
                    sig_we <= 1'b1;
                    sig_wa <= out_cnt;
                    sig_wd <= {fft_or[OWIDTH-1:OWIDTH-BW], fft_oi[OWIDTH-1:OWIDTH-BW]};
                end

                if (out_active && out_cnt == N[LGN-1:0] - 1) begin
                    cnt           <= 0;
                    fft_rst_r     <= 1'b1;
                    out_sync_seen <= 1'b0;
                    out_active    <= 1'b0;
                    state         <= S_FFT_MUL;
                end
            end

            // ============================================================
            // Pass 2: FFT code, multiply with sig_buf, write conj(product)
            S_FFT_MUL: begin
                fft_ce <= 1'b1;

                if (cnt < N) begin
                    if (cnt < CODE_LEN)
                        fft_in <= code_rom[cnt[9:0]][0]
                                  ? {4'sb0001, 4'sb0000}
                                  : {4'sb1111, 4'sb0000};
                end

                cnt <= cnt + 1;

                if (out_active) begin
                    pipe_cr       <= fft_or;
                    pipe_ci       <= fft_oi;
                    pipe_mul_addr <= out_cnt;
                    sig_ra        <= out_cnt;
                end

                if (out_active && out_cnt > 0)
                    pipe_mul_valid <= 1'b1;

                if (!out_active && out_sync_seen && !pipe_mul_valid
                    && pipe_mul_addr == N[LGN-1:0] - 1) begin
                    pipe_mul_valid <= 1'b1;
                    cnt            <= 0;
                    fft_rst_r      <= 1'b1;
                    out_sync_seen  <= 1'b0;
                    out_active     <= 1'b0;
                    sig_ra         <= bitrev(12'd0);
                    state          <= S_FFT_IFFT;
                end
            end

            // ============================================================
            // Pass 3: FFT(conj(P)) → IFFT → accumulate |I|+|Q|
            S_FFT_IFFT: begin
                fft_ce <= 1'b1;

                if (cnt < N) begin
                    fft_in <= {sig_rd[2*BW-1:2*BW-IWIDTH],
                               sig_rd[BW-1:BW-IWIDTH]};
                    if (cnt < N - 1)
                        sig_ra <= bitrev(cnt[LGN-1:0] + 1);
                end

                cnt <= cnt + 1;

                if (out_active) begin
                    pipe_mag      <= mag;
                    pipe_acc_addr <= bitrev(out_cnt);
                    pwr_ra        <= bitrev(out_cnt);
                end

                if (out_active && out_cnt > 0)
                    pipe_acc_valid <= 1'b1;

                if (!out_active && out_sync_seen && !pipe_acc_valid
                    && pipe_acc_addr == bitrev(N[LGN-1:0] - 1)) begin
                    pipe_acc_valid <= 1'b1;
                    state          <= S_NEXT_EPOCH;
                end
            end

            // ============================================================
            S_NEXT_EPOCH: begin
                epoch_cnt <= epoch_cnt + 6'd1;
                if (epoch_cnt + 6'd1 == num_epochs) begin
                    scan_addr  <= 0;
                    scan_pipe  <= 2'd0;
                    best_phase <= 10'd0;
                    best_power <= 32'd0;
                    pwr_ra     <= 0;
                    state      <= S_FIND_BEST;
                end else begin
                    cnt   <= 0;
                    state <= S_CAPTURE;
                end
            end

            // ============================================================
            S_FIND_BEST: begin
                case (scan_pipe)
                2'd0: scan_pipe <= 2'd1;
                2'd1: begin
                    if ({16'd0, pwr_rd} > best_power) begin
                        best_power <= {16'd0, pwr_rd};
                        best_phase <= scan_addr[LGN-1:2];
                    end
                    if (scan_addr == N[LGN-1:0] - 1) begin
                        state <= S_DONE;
                    end else begin
                        scan_addr <= scan_addr + 1;
                        pwr_ra    <= scan_addr + 1;
                        scan_pipe <= 2'd0;
                    end
                end
                default: scan_pipe <= 2'd0;
                endcase
            end

            // ============================================================
            S_DONE: ;

            endcase
        end
    end

endmodule
