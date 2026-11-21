// GPS matched-filter acquisition engine
//
// Captures one epoch of carrier-wiped I/Q into block RAM, then correlates
// against all 1023 code phases using LANES parallel banks per pass.
// Reports the best code phase and its |I|+|Q| magnitude.
//
// Timing (LANES=20):
//   Code generation:   1023 cycles
//   Capture:           4096 sample_valid events (1 ms at 4.096 MS/s)
//   Processing:        ceil(1023/LANES) × (SR_LEN + 4096) cycles
//   Total processing:  ~270K cycles → 66 ms at 4.096 MHz, 3.5 ms at 77.5 MHz
//
// Resources (LANES=20):
//   ~1050 FFs (shift register) + ~1000 FFs (accumulators) + ~1500 LUTs
//   ~6 EBR (epoch buffer)

module gps_acq_engine #(
    parameter LANES = 20
) (
    input  wire        clk,
    input  wire        rst,

    input  wire signed [11:0] i_in,
    input  wire signed [11:0] q_in,
    input  wire        sample_valid,

    input  wire        start,
    input  wire        abort,
    input  wire [4:0]  prn,
    input  wire [31:0] carrier_freq,

    output wire        done,
    output wire        active,
    output reg  [9:0]  best_phase,
    output reg  [31:0] best_power
);

    localparam CODE_LEN      = 1023;
    localparam EPOCH_SAMPLES = 4096;
    localparam SR_LEN        = CODE_LEN + LANES - 1;
    localparam NUM_BATCHES   = (CODE_LEN + LANES - 1) / LANES;
    localparam CODE_FREQ     = 32'h3FF00000;

    localparam S_IDLE      = 3'd0,
               S_GEN_CODE  = 3'd1,
               S_CAPTURE   = 3'd2,
               S_LOAD_SR   = 3'd3,
               S_CORRELATE = 3'd4,
               S_PEAK      = 3'd5,
               S_DONE      = 3'd6;

    reg [2:0] state;
    assign done   = (state == S_DONE);
    assign active = (state != S_IDLE) & (state != S_DONE);

    // ---- Carrier NCO (capture phase) ----
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

    // ---- C/A code generator ----
    wire ca_gen_rst = rst | (state == S_IDLE);
    reg  ca_chip_en;
    wire ca_code_out, ca_epoch_out, ca_busy;

    gps_ca_gen ca_gen (
        .clk       (clk),
        .rst       (ca_gen_rst),
        .prn       (prn),
        .chip_en   (ca_chip_en),
        .phase_wen (1'b0),
        .phase_in  (10'd0),
        .code_out  (ca_code_out),
        .epoch     (ca_epoch_out),
        .busy      (ca_busy)
    );

    // Code ROM (register array, 1023 × 1 bit)
    reg code_orig [0:CODE_LEN-1];
    reg [9:0] gen_ptr;

    // ---- Epoch buffer (inferred dual-port block RAM) ----
    reg [23:0] epoch_buf [0:EPOCH_SAMPLES-1];
    reg [11:0] buf_waddr, buf_raddr;
    reg [23:0] buf_rdata;

    always @(posedge clk)
        buf_rdata <= epoch_buf[buf_raddr];

    // ---- Code shift register ----
    reg [SR_LEN-1:0] code_sr;

    // ---- Per-lane accumulators ----
    reg signed [23:0] acc_i [0:LANES-1];
    reg signed [23:0] acc_q [0:LANES-1];

    // ---- Processing counters ----
    reg [5:0]  batch_idx;
    reg [9:0]  batch_base;
    reg [12:0] sample_cnt;
    reg [10:0] load_ptr;
    reg [9:0]  load_code_idx;

    // Code NCO for chip tracking during correlation
    reg [31:0] proc_code_phase;
    wire [32:0] proc_code_sum = {1'b0, proc_code_phase} + {1'b0, CODE_FREQ};
    wire proc_chip_carry = proc_code_sum[32];

    // Pipeline: code bits captured one cycle before accumulation
    reg [LANES-1:0] code_bits_d1;
    reg              corr_valid;

    // Peak scan
    reg [4:0] peak_idx;

    // ---- Main FSM ----
    integer j;
    always @(posedge clk) begin
        if (rst) begin
            state     <= S_IDLE;
            car_phase <= 32'd0;
            ca_chip_en <= 1'b0;
            gen_ptr   <= 10'd0;
            buf_waddr <= 12'd0;
            batch_idx <= 6'd0;
            batch_base <= 10'd0;
            best_phase <= 10'd0;
            best_power <= 32'd0;
            corr_valid <= 1'b0;
        end else if (abort && state != S_IDLE) begin
            state <= S_IDLE;
        end else begin
            case (state)

            // ---------------------------------------------------
            S_IDLE: begin
                ca_chip_en <= 1'b0;
                corr_valid <= 1'b0;
                if (start) begin
                    gen_ptr <= 10'd0;
                    ca_chip_en <= 1'b0;
                    state <= S_GEN_CODE;
                end
            end

            // ---------------------------------------------------
            S_GEN_CODE: begin
                code_orig[gen_ptr] <= ca_code_out;
                ca_chip_en <= (gen_ptr < CODE_LEN - 2);
                gen_ptr <= gen_ptr + 10'd1;
                if (gen_ptr == CODE_LEN - 1) begin
                    ca_chip_en <= 1'b0;
                    car_phase  <= 32'd0;
                    buf_waddr  <= 12'd0;
                    state      <= S_CAPTURE;
                end
            end

            // ---------------------------------------------------
            S_CAPTURE: begin
                if (sample_valid) begin
                    epoch_buf[buf_waddr] <= {q_mix[13:2], i_mix[13:2]};
                    car_phase <= car_phase + carrier_freq;
                    if (buf_waddr == EPOCH_SAMPLES - 1) begin
                        batch_idx  <= 6'd0;
                        batch_base <= 10'd0;
                        best_phase <= 10'd0;
                        best_power <= 32'd0;
                        load_ptr   <= 11'd0;
                        load_code_idx <= 10'd0;
                        state      <= S_LOAD_SR;
                    end
                    buf_waddr <= buf_waddr + 12'd1;
                end
            end

            // ---------------------------------------------------
            S_LOAD_SR: begin
                // Shift in code bits from MSB side: after SR_LEN cycles,
                // code_sr[k] = code_orig[(batch_base + k) % 1023]
                code_sr <= {code_orig[load_code_idx], code_sr[SR_LEN-1:1]};
                load_code_idx <= (load_code_idx == CODE_LEN - 1)
                                   ? 10'd0 : load_code_idx + 10'd1;
                load_ptr <= load_ptr + 11'd1;
                if (load_ptr == SR_LEN - 1) begin
                    sample_cnt      <= 13'd0;
                    buf_raddr       <= 12'd0;
                    proc_code_phase <= 32'd0;
                    corr_valid      <= 1'b0;
                    for (j = 0; j < LANES; j = j + 1) begin
                        acc_i[j] <= 24'd0;
                        acc_q[j] <= 24'd0;
                    end
                    state <= S_CORRELATE;
                end
            end

            // ---------------------------------------------------
            S_CORRELATE: begin
                // Pipeline stage 1: capture code bits and issue next read
                for (j = 0; j < LANES; j = j + 1)
                    code_bits_d1[j] <= code_sr[j];
                buf_raddr <= sample_cnt[11:0] + 12'd1;
                proc_code_phase <= proc_code_sum[31:0];
                if (proc_chip_carry)
                    code_sr <= code_sr >> 1;

                // Pipeline stage 2: accumulate with previous cycle's data
                if (corr_valid) begin
                    for (j = 0; j < LANES; j = j + 1) begin
                        if (code_bits_d1[j]) begin
                            acc_i[j] <= acc_i[j] + {{12{buf_rdata[11]}}, buf_rdata[11:0]};
                            acc_q[j] <= acc_q[j] + {{12{buf_rdata[23]}}, buf_rdata[23:12]};
                        end else begin
                            acc_i[j] <= acc_i[j] - {{12{buf_rdata[11]}}, buf_rdata[11:0]};
                            acc_q[j] <= acc_q[j] - {{12{buf_rdata[23]}}, buf_rdata[23:12]};
                        end
                    end
                end
                corr_valid <= 1'b1;

                sample_cnt <= sample_cnt + 13'd1;
                if (sample_cnt == EPOCH_SAMPLES) begin
                    peak_idx   <= 5'd0;
                    corr_valid <= 1'b0;
                    state      <= S_PEAK;
                end
            end

            // ---------------------------------------------------
            S_PEAK: begin
                // Scan LANES accumulators, update best if stronger
                begin : peak_check
                    reg signed [23:0] pi, pq;
                    reg [23:0] ai, aq;
                    reg [24:0] mag;
                    reg [9:0]  ph;
                    pi = acc_i[peak_idx];
                    pq = acc_q[peak_idx];
                    ai = pi[23] ? (~pi + 24'd1) : pi;
                    aq = pq[23] ? (~pq + 24'd1) : pq;
                    mag = {1'b0, ai} + {1'b0, aq};
                    ph = batch_base + {5'd0, peak_idx};

                    if (ph < CODE_LEN && {7'd0, mag} > best_power) begin
                        best_power <= {7'd0, mag};
                        best_phase <= ph;
                    end
                end

                peak_idx <= peak_idx + 5'd1;
                if (peak_idx == LANES - 1) begin
                    if (batch_idx == NUM_BATCHES - 1) begin
                        state <= S_DONE;
                    end else begin
                        batch_idx  <= batch_idx + 6'd1;
                        batch_base <= batch_base + LANES[9:0];
                        load_ptr   <= 11'd0;
                        // load_code_idx continues wrapping from previous batch
                        state <= S_LOAD_SR;
                    end
                end
            end

            // ---------------------------------------------------
            S_DONE: ;

            endcase
        end
    end

endmodule
