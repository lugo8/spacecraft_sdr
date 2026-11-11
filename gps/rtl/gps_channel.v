// Single GPS correlator channel — carrier wipe-off, E/P/L code correlation, accumulate-and-dump
module gps_channel (
    input  wire        clk,
    input  wire        rst,
    input  wire        enable,
    input  wire [4:0]  prn,
    input  wire        sample_valid,
    input  wire signed [11:0] i_in,
    input  wire signed [11:0] q_in,
    input  wire [31:0] carrier_freq,
    input  wire [31:0] code_freq,

    // Code phase preset (for acquisition)
    input  wire        phase_wen,
    input  wire [9:0]  phase_in,

    output reg  signed [31:0] ie_out, qe_out,
    output reg  signed [31:0] ip_out, qp_out,
    output reg  signed [31:0] il_out, ql_out,
    output reg         epoch_flag,
    output reg  [15:0] epoch_count,
    output wire        busy
);

    // ---- Carrier NCO ----
    reg [31:0] car_phase;

    // 2-bit quadrant carrier wipe-off (complex multiply with sign-only LO)
    // cos > 0 in Q0,Q3; sin > 0 in Q0,Q1
    wire cos_neg = car_phase[31] ^ car_phase[30];
    wire sin_neg = car_phase[31];

    wire signed [12:0] i_ext = {i_in[11], i_in};
    wire signed [12:0] q_ext = {q_in[11], q_in};

    // I_out = I*cos + Q*sin,  Q_out = Q*cos - I*sin
    wire signed [12:0] i_x_cos = cos_neg ? -i_ext : i_ext;
    wire signed [12:0] q_x_sin = sin_neg ? -q_ext : q_ext;
    wire signed [12:0] q_x_cos = cos_neg ? -q_ext : q_ext;
    wire signed [12:0] i_x_sin = sin_neg ? -i_ext : i_ext;

    wire signed [13:0] i_mix = i_x_cos + q_x_sin;
    wire signed [13:0] q_mix = q_x_cos - i_x_sin;

    // ---- Code NCO ----
    reg [31:0] code_phase;
    wire [32:0] code_sum   = {1'b0, code_phase} + {1'b0, code_freq};
    wire        code_carry = code_sum[32];
    wire        chip_en    = sample_valid & code_carry & ~busy;

    // ---- C/A code generator ----
    wire ca_code, ca_epoch;
    gps_ca_gen ca_gen (
        .clk       (clk),
        .rst       (rst | ~enable),
        .prn       (prn),
        .chip_en   (chip_en),
        .phase_wen (phase_wen),
        .phase_in  (phase_in),
        .code_out  (ca_code),
        .epoch     (ca_epoch),
        .busy      (busy)
    );

    // E/P/L from delay line (2-chip E-L spacing)
    reg code_prompt, code_late;
    always @(posedge clk) begin
        if (rst | ~enable) begin
            code_prompt <= 1'b0;
            code_late   <= 1'b0;
        end else if (chip_en) begin
            code_prompt <= ca_code;
            code_late   <= code_prompt;
        end
    end
    wire code_early = ca_code;

    // ---- Correlate: code_bit ? +sample : -sample ----
    wire signed [13:0] neg_i_mix = -i_mix;
    wire signed [13:0] neg_q_mix = -q_mix;

    wire signed [13:0] corr_ie = code_early  ? i_mix : neg_i_mix;
    wire signed [13:0] corr_qe = code_early  ? q_mix : neg_q_mix;
    wire signed [13:0] corr_ip = code_prompt ? i_mix : neg_i_mix;
    wire signed [13:0] corr_qp = code_prompt ? q_mix : neg_q_mix;
    wire signed [13:0] corr_il = code_late   ? i_mix : neg_i_mix;
    wire signed [13:0] corr_ql = code_late   ? q_mix : neg_q_mix;

    // Sign-extend to 32 bits
    wire signed [31:0] corr_ie_x = {{18{corr_ie[13]}}, corr_ie};
    wire signed [31:0] corr_qe_x = {{18{corr_qe[13]}}, corr_qe};
    wire signed [31:0] corr_ip_x = {{18{corr_ip[13]}}, corr_ip};
    wire signed [31:0] corr_qp_x = {{18{corr_qp[13]}}, corr_qp};
    wire signed [31:0] corr_il_x = {{18{corr_il[13]}}, corr_il};
    wire signed [31:0] corr_ql_x = {{18{corr_ql[13]}}, corr_ql};

    // ---- Accumulators ----
    reg signed [31:0] acc_ie, acc_qe, acc_ip, acc_qp, acc_il, acc_ql;

    wire signed [31:0] next_ie = acc_ie + corr_ie_x;
    wire signed [31:0] next_qe = acc_qe + corr_qe_x;
    wire signed [31:0] next_ip = acc_ip + corr_ip_x;
    wire signed [31:0] next_qp = acc_qp + corr_qp_x;
    wire signed [31:0] next_il = acc_il + corr_il_x;
    wire signed [31:0] next_ql = acc_ql + corr_ql_x;

    always @(posedge clk) begin
        if (rst | ~enable) begin
            car_phase   <= 32'd0;
            code_phase  <= 32'd0;
            acc_ie <= 0; acc_qe <= 0;
            acc_ip <= 0; acc_qp <= 0;
            acc_il <= 0; acc_ql <= 0;
            ie_out <= 0; qe_out <= 0;
            ip_out <= 0; qp_out <= 0;
            il_out <= 0; ql_out <= 0;
            epoch_flag  <= 1'b0;
            epoch_count <= 16'd0;
        end else begin
            epoch_flag <= 1'b0;

            if (sample_valid & ~busy) begin
                car_phase  <= car_phase + carrier_freq;
                code_phase <= code_sum[31:0];

                if (ca_epoch) begin
                    ie_out <= next_ie; qe_out <= next_qe;
                    ip_out <= next_ip; qp_out <= next_qp;
                    il_out <= next_il; ql_out <= next_ql;
                    acc_ie <= 0; acc_qe <= 0;
                    acc_ip <= 0; acc_qp <= 0;
                    acc_il <= 0; acc_ql <= 0;
                    epoch_flag  <= 1'b1;
                    epoch_count <= epoch_count + 16'd1;
                end else begin
                    acc_ie <= next_ie; acc_qe <= next_qe;
                    acc_ip <= next_ip; acc_qp <= next_qp;
                    acc_il <= next_il; acc_ql <= next_ql;
                end
            end
        end
    end

endmodule
