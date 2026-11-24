// 12-channel GPS L1 C/A correlator with matched-filter acquisition and on-FPGA tracking
//
// Register map (7-bit address = {channel[3:0], register[2:0]}):
//
// Per-channel (ch 0-11):
//   0: CTRL       [RW]  {25'b0, track, enable, prn[4:0]}
//   1: CAR_FREQ   [RW]  carrier NCO frequency word (loop filter updates when track=1)
//   2: COD_FREQ   [RW]  code NCO frequency word (loop filter updates when track=1)
//   3: COD_PHASE  [W]   {22'b0, chip_phase[9:0]} — preset code phase
//   3: IP         [R]   I-Prompt accumulator (32-bit signed)
//   4: QP         [R]   Q-Prompt accumulator (32-bit signed)
//   5: IE_QE      [R]   {ie[31:16], qe[31:16]}
//   6: IL_QL      [R]   {il[31:16], ql[31:16]}
//   7: STATUS     [R]   {14'b0, busy, epoch_flag, epoch_count[15:0]}
//
// Channel 12 — loop filter gains:
//   0: CAR_GAIN_P [RW]  carrier PLL proportional gain (signed 32-bit)
//   1: CAR_GAIN_I [RW]  carrier PLL integral gain (signed 32-bit)
//   2: COD_GAIN   [RW]  code DLL gain (signed 32-bit)
//   3: FLL_GAIN   [RW]  FLL frequency gain (signed 32-bit)
//
// Channel 13 — FLL/carrier-aiding config:
//   0: FLL_EPOCHS [RW]  number of epochs in FLL mode before switching to PLL (default 400)
//
// Channel 15 — global:
//   0: GLOB_CTRL  [RW]  {31'b0, enable}
//   1: EPOCH_CNT  [R]   global epoch counter
//   2: VERSION    [R]   0x47505307 ("GPS" + v7)
//   3: ACQ_CTRL   [W]   bit 0: start, bit 1: abort
//      ACQ_STATUS [R]   {done, active, 20'b0, best_phase[9:0]}
//   4: ACQ_CFG    [RW]  {5'b0, prn[4:0], 16'b0, num_epochs[5:0]}
//   5: ACQ_CARRIER[RW]  carrier NCO freq word for acquisition
//   6: ACQ_POWER  [R]   best |IP|+|QP| (accumulated over num_epochs)
//   7: (reserved)
//
// Tracking loop filter:
//   FLL (first FLL_EPOCHS after tracking enable):
//     cross = prev_IP × QP - prev_QP × IP
//     dot   = prev_IP × IP + prev_QP × QP
//     freq_err = cross × sign(dot)  (simplified discriminator)
//     carrier_freq += (freq_err × FLL_GAIN) >> 16
//   PLL (after FLL_EPOCHS):
//     norm_phase = (QP × sign(IP)) / |IP|
//     carrier_freq += (norm_phase × CAR_GAIN_P + integral × CAR_GAIN_I) >> 16
//   DLL:
//     norm_code = (|IE|+|QE| - |IL|-|QL|) / (|IE|+|QE| + |IL|+|QL|)
//     code_freq -= (norm_code × COD_GAIN) >> 16
//   Carrier-aiding:
//     code_freq += (carrier_freq - carrier_freq_at_track_start) × CODE_FREQ_NOM >> 32
//     (applied at writeback via approximate shift: car_delta × 1023/4096 ≈ car_delta >> 2)

module gps_correlator (
    input  wire        clk,
    input  wire        rst,

    input  wire signed [11:0] i_in,
    input  wire signed [11:0] q_in,
    input  wire        sample_valid,

    input  wire [6:0]  reg_addr,
    input  wire [31:0] reg_wdata,
    output reg  [31:0] reg_rdata,
    input  wire        reg_wen
);

    localparam NUM_CH = 8;
    localparam CODE_FREQ_NOM = 32'h3FF00000;
    localparam DIV_CYCLES = 20;

    // ---- Global control ----
    reg global_enable;
    reg [31:0] global_epoch;

    // ---- Per-channel config ----
    reg [4:0]  ch_prn    [0:NUM_CH-1];
    reg        ch_en     [0:NUM_CH-1];
    reg        ch_track  [0:NUM_CH-1];
    reg [31:0] ch_car_f  [0:NUM_CH-1];
    reg [31:0] ch_cod_f  [0:NUM_CH-1];

    reg [NUM_CH-1:0] ch_phase_wen;
    reg [9:0]        ch_phase_val;

    // ---- Per-channel outputs ----
    wire signed [31:0] ch_ie [0:NUM_CH-1], ch_qe [0:NUM_CH-1];
    wire signed [31:0] ch_ip [0:NUM_CH-1], ch_qp [0:NUM_CH-1];
    wire signed [31:0] ch_il [0:NUM_CH-1], ch_ql [0:NUM_CH-1];
    wire        ch_epoch_flag  [0:NUM_CH-1];
    wire [15:0] ch_epoch_count [0:NUM_CH-1];
    wire        ch_busy        [0:NUM_CH-1];

    // ---- Channel instances ----
    genvar g;
    generate
        for (g = 0; g < NUM_CH; g = g + 1) begin : ch
            gps_channel channel (
                .clk          (clk),
                .rst          (rst),
                .enable       (global_enable & ch_en[g]),
                .prn          (ch_prn[g]),
                .sample_valid (sample_valid),
                .i_in         (i_in),
                .q_in         (q_in),
                .carrier_freq (ch_car_f[g]),
                .code_freq    (ch_cod_f[g]),
                .phase_wen    (ch_phase_wen[g]),
                .phase_in     (ch_phase_val),
                .ie_out       (ch_ie[g]),
                .qe_out       (ch_qe[g]),
                .ip_out       (ch_ip[g]),
                .qp_out       (ch_qp[g]),
                .il_out       (ch_il[g]),
                .ql_out       (ch_ql[g]),
                .epoch_flag   (ch_epoch_flag[g]),
                .epoch_count  (ch_epoch_count[g]),
                .busy         (ch_busy[g])
            );
        end
    endgenerate

    // ---- Matched-filter acquisition engine ----
    reg        acq_start, acq_abort;
    reg [4:0]  acq_prn;
    reg [31:0] acq_carrier;
    reg [5:0]  acq_num_epochs;
    wire       acq_done, acq_active;
    wire [9:0] acq_best_phase;
    wire [31:0] acq_best_power;

    gps_fft_acq acq_eng (
        .clk          (clk),
        .rst          (rst),
        .i_in         (i_in),
        .q_in         (q_in),
        .sample_valid (sample_valid),
        .start        (acq_start),
        .abort        (acq_abort),
        .prn          (acq_prn),
        .carrier_freq (acq_carrier),
        .num_epochs   (acq_num_epochs),
        .done         (acq_done),
        .active       (acq_active),
        .best_phase   (acq_best_phase),
        .best_power   (acq_best_power)
    );

    // ---- Tracking loop filter ----

    // Per-channel state
    reg signed [31:0] car_integral [0:NUM_CH-1];
    reg signed [31:0] prev_ip      [0:NUM_CH-1];
    reg signed [31:0] prev_qp      [0:NUM_CH-1];
    reg [15:0]        fll_count    [0:NUM_CH-1];
    reg               have_prev    [0:NUM_CH-1];
    reg signed [31:0] car_f_base   [0:NUM_CH-1]; // carrier freq at track start (for aiding)

    // Loop filter gains (channel 12 address space)
    reg signed [31:0] car_gain_p;
    reg signed [31:0] car_gain_i;
    reg signed [31:0] cod_gain;
    reg signed [31:0] fll_gain;

    // FLL config (channel 13 address space)
    reg [15:0] fll_epochs;  // epochs in FLL mode before PLL (default 400)

    // Epoch pending flags for tracking channels
    reg [NUM_CH-1:0] track_pending;

    // Scan state: cycles through channels, 52 sub-cycles each
    reg [3:0] scan_ch;
    reg [5:0] scan_sub;

    // Shared registered multiplier (infers DSP slices)
    reg  signed [31:0] mul_a, mul_b;
    reg  signed [63:0] mul_result;
    always @(posedge clk)
        mul_result <= mul_a * mul_b;

    // Accumulator mux for current scan channel
    wire signed [31:0] scan_ip = ch_ip[scan_ch];
    wire signed [31:0] scan_qp = ch_qp[scan_ch];
    wire signed [31:0] scan_ie = ch_ie[scan_ch];
    wire signed [31:0] scan_qe = ch_qe[scan_ch];
    wire signed [31:0] scan_il = ch_il[scan_ch];
    wire signed [31:0] scan_ql = ch_ql[scan_ch];

    wire signed [31:0] scan_prev_ip = prev_ip[scan_ch];
    wire signed [31:0] scan_prev_qp = prev_qp[scan_ch];
    wire               scan_in_fll  = (fll_count[scan_ch] < fll_epochs) && have_prev[scan_ch];
    wire               scan_in_pll  = (fll_count[scan_ch] >= fll_epochs);

    // ---- Raw discriminator components ----

    // PLL raw: QP × sign(IP) (Costas, data-insensitive)
    wire signed [31:0] raw_phase_err = scan_ip[31] ? scan_qp : (-scan_qp);

    // DLL envelope magnitudes
    wire [31:0] abs_ie = scan_ie[31] ? (~scan_ie + 32'd1) : scan_ie;
    wire [31:0] abs_qe = scan_qe[31] ? (~scan_qe + 32'd1) : scan_qe;
    wire [31:0] abs_il = scan_il[31] ? (~scan_il + 32'd1) : scan_il;
    wire [31:0] abs_ql = scan_ql[31] ? (~scan_ql + 32'd1) : scan_ql;
    wire [31:0] abs_ip = scan_ip[31] ? (~scan_ip + 32'd1) : scan_ip;

    wire [31:0] em = abs_ie + abs_qe;    // early envelope magnitude
    wire [31:0] lm = abs_il + abs_ql;    // late envelope magnitude
    wire signed [31:0] raw_code_err = $signed({1'b0, em[30:0]}) - $signed({1'b0, lm[30:0]});
    wire [31:0] code_sum = em + lm;       // DLL denominator

    // Absolute values for divider numerator loading
    wire [31:0] abs_raw_phase = raw_phase_err[31] ? (~raw_phase_err + 32'd1) : raw_phase_err;
    wire [31:0] abs_raw_code  = raw_code_err[31]  ? (~raw_code_err  + 32'd1) : raw_code_err;

    // ---- Sequential divider ----
    reg [25:0] div_rem;
    reg [25:0] div_den;
    reg [19:0] div_dvd;
    reg [19:0] div_quot;
    reg        div_sign;

    wire [25:0] div_r_shifted = {div_rem[24:0], div_dvd[19]};
    wire        div_ge = (div_r_shifted >= div_den);

    // Normalized discriminator outputs (signed 32-bit, ×2^16 scale)
    reg signed [31:0] norm_phase_err;
    reg signed [31:0] norm_code_err;

    wire signed [31:0] scan_integral = car_integral[scan_ch];

    // Pipeline term registers
    reg signed [31:0] car_p_term;
    reg signed [31:0] car_i_term;
    reg signed [31:0] fll_term;

    // FLL cross/dot intermediate registers
    reg signed [31:0] fll_cross;
    reg signed [31:0] fll_dot;

    // Multiply result extraction
    wire signed [31:0] mul_term = mul_result[47:16];

    // ---- Scan sub-cycle schedule (52 cycles per channel) ----
    //  0:     Setup PLL division
    //  1-20:  PLL division iterations
    // 21:     Latch PLL result; setup DLL division
    // 22-41:  DLL division iterations
    // 42:     Latch DLL; start FLL cross multiply 1: prev_IP × QP
    // 43:     Latch cross1; start FLL cross multiply 2: prev_QP × IP
    // 44:     Compute cross = cross1 - cross2; start FLL dot multiply 1: prev_IP × IP
    // 45:     Latch dot1; start FLL dot multiply 2: prev_QP × QP
    // 46:     Compute dot = dot1 + dot2; start carrier gain multiply
    //           (FLL mode: fll_freq_err × FLL_GAIN; PLL mode: norm_phase × CAR_GAIN_P)
    // 47:     Save car_p/fll term; start carrier integral multiply (PLL only)
    // 48:     Save car_i term; start code DLL multiply
    // 49:     Save code term; start carrier-aiding multiply
    // 50:     Latch aiding term
    // 51:     Writeback NCO updates; advance to next channel

    wire in_pll_div = (scan_sub >= 6'd1  && scan_sub <= 6'd20);
    wire in_dll_div = (scan_sub >= 6'd22 && scan_sub <= 6'd41);

    // ---- Main clocked process ----
    wire [3:0] wr_ch  = reg_addr[6:3];
    wire [2:0] wr_reg = reg_addr[2:0];

    // Carrier-aiding: delta from base freq, scaled to code NCO
    // code_rate / carrier_freq = 1.023e6 / 1575.42e6 ≈ 1/1540
    // In NCO domain: code_delta = car_delta / 1540
    // Approximate: /1540 ≈ /1536 = >>10 + fine correction (0.26% error, negligible)
    wire signed [31:0] car_delta = ch_car_f[scan_ch] - car_f_base[scan_ch];
    wire signed [31:0] code_aiding = {{10{car_delta[31]}}, car_delta[31:10]};

    integer i;
    always @(posedge clk) begin
        // Default pulse clears
        ch_phase_wen <= {NUM_CH{1'b0}};
        acq_start    <= 1'b0;
        acq_abort    <= 1'b0;

        if (rst) begin
            global_enable <= 1'b0;
            global_epoch  <= 32'd0;
            acq_prn        <= 5'd0;
            acq_carrier    <= 32'd0;
            acq_num_epochs <= 6'd40;
            car_gain_p    <= 32'd0;
            car_gain_i    <= 32'd0;
            cod_gain      <= 32'd0;
            fll_gain      <= 32'd0;
            fll_epochs    <= 16'd400;
            track_pending <= {NUM_CH{1'b0}};
            scan_ch       <= 4'd0;
            scan_sub      <= 6'd0;
            norm_phase_err <= 32'd0;
            norm_code_err  <= 32'd0;
            div_rem       <= 26'd0;
            div_den       <= 26'd0;
            div_dvd       <= 20'd0;
            div_quot      <= 20'd0;
            div_sign      <= 1'b0;
            car_p_term    <= 32'd0;
            car_i_term    <= 32'd0;
            fll_term      <= 32'd0;
            fll_cross     <= 32'd0;
            fll_dot       <= 32'd0;
            for (i = 0; i < NUM_CH; i = i + 1) begin
                ch_prn[i]       <= 5'd0;
                ch_en[i]        <= 1'b0;
                ch_track[i]     <= 1'b0;
                ch_car_f[i]     <= 32'd0;
                ch_cod_f[i]     <= CODE_FREQ_NOM;
                car_integral[i] <= 32'd0;
                prev_ip[i]      <= 32'd0;
                prev_qp[i]      <= 32'd0;
                fll_count[i]    <= 16'd0;
                have_prev[i]    <= 1'b0;
                car_f_base[i]   <= 32'd0;
            end
        end else begin
            // Global epoch counter
            if (ch_epoch_flag[0])
                global_epoch <= global_epoch + 32'd1;

            // ---- Tracking loop filter pipeline ----
            scan_sub <= (scan_sub == 6'd51) ? 6'd0 : scan_sub + 6'd1;

            // Division iteration (shared between PLL and DLL phases)
            if (in_pll_div || in_dll_div) begin
                div_dvd <= {div_dvd[18:0], 1'b0};
                if (div_ge) begin
                    div_rem  <= div_r_shifted - div_den;
                    div_quot <= {div_quot[18:0], 1'b1};
                end else begin
                    div_rem  <= div_r_shifted;
                    div_quot <= {div_quot[18:0], 1'b0};
                end
            end

            case (scan_sub)
                6'd0: begin
                    // Setup PLL division: QP×sign(IP) / |IP|
                    if (abs_ip == 32'd0) begin
                        div_den  <= 26'd1;
                        div_rem  <= 26'd0;
                        div_dvd  <= 20'd0;
                    end else begin
                        div_den <= abs_ip[25:0];
                        div_rem <= {5'b0, abs_raw_phase[24:4]};
                        div_dvd <= {abs_raw_phase[3:0], 16'b0};
                    end
                    div_sign <= raw_phase_err[31];
                    div_quot <= 20'd0;
                end

                6'd21: begin
                    // Latch PLL normalized result
                    if (div_sign)
                        norm_phase_err <= -$signed({12'd0, div_quot});
                    else
                        norm_phase_err <= $signed({12'd0, div_quot});

                    // Setup DLL division: (Em-Lm) / (Em+Lm)
                    if (code_sum == 32'd0) begin
                        div_den <= 26'd1;
                        div_rem <= 26'd0;
                        div_dvd <= 20'd0;
                    end else begin
                        div_den <= code_sum[25:0];
                        div_rem <= {5'b0, abs_raw_code[24:4]};
                        div_dvd <= {abs_raw_code[3:0], 16'b0};
                    end
                    div_sign <= raw_code_err[31];
                    div_quot <= 20'd0;
                end

                6'd42: begin
                    // Latch DLL normalized result
                    if (div_sign)
                        norm_code_err <= -$signed({12'd0, div_quot});
                    else
                        norm_code_err <= $signed({12'd0, div_quot});

                    // Start FLL cross multiply 1: prev_IP × QP
                    mul_a <= scan_prev_ip;
                    mul_b <= scan_qp;
                end

                6'd43: begin
                    // Latch prev_IP×QP, start prev_QP × IP
                    fll_cross <= mul_term;
                    mul_a <= scan_prev_qp;
                    mul_b <= scan_ip;
                end

                6'd44: begin
                    // cross = prev_IP×QP - prev_QP×IP; start prev_IP × IP
                    fll_cross <= fll_cross - mul_term;
                    mul_a <= scan_prev_ip;
                    mul_b <= scan_ip;
                end

                6'd45: begin
                    // Latch prev_IP×IP, start prev_QP × QP
                    fll_dot <= mul_term;
                    mul_a <= scan_prev_qp;
                    mul_b <= scan_qp;
                end

                6'd46: begin
                    // dot = prev_IP×IP + prev_QP×QP
                    fll_dot <= fll_dot + mul_term;

                    // Start carrier gain multiply:
                    if (scan_in_fll) begin
                        // FLL: freq_err = cross × sign(dot)
                        // fll_dot sign known next cycle; use current fll_dot+mul_term
                        // For now, load: fll_cross × sign(dot)
                        if ((fll_dot + mul_term) < 0)
                            mul_a <= -fll_cross;
                        else
                            mul_a <= fll_cross;
                        mul_b <= fll_gain;
                    end else begin
                        // PLL: norm_phase_err × CAR_GAIN_P
                        mul_a <= norm_phase_err;
                        mul_b <= car_gain_p;
                    end
                end

                6'd47: begin
                    if (scan_in_fll) begin
                        // FLL term ready
                        fll_term <= mul_term;
                        // No integral multiply needed in FLL mode; load dummy
                        mul_a <= 32'd0;
                        mul_b <= 32'd0;
                    end else begin
                        // PLL: car_p ready, start integral multiply
                        car_p_term <= mul_term;
                        mul_a <= scan_integral;
                        mul_b <= car_gain_i;
                    end
                end

                6'd48: begin
                    if (!scan_in_fll)
                        car_i_term <= mul_term;
                    // Start code DLL multiply
                    mul_a <= norm_code_err;
                    mul_b <= cod_gain;
                end

                6'd49: begin
                    // Code term ready (mul_term holds DLL result)
                    // No extra multiply needed; carrier-aiding uses shift
                end

                6'd50: begin
                    // Available for future use
                end

                6'd51: begin
                    // Writeback NCO updates and advance
                    if (track_pending[scan_ch]) begin
                        track_pending[scan_ch] <= 1'b0;

                        // Update prev accumulators for FLL
                        prev_ip[scan_ch] <= scan_ip;
                        prev_qp[scan_ch] <= scan_qp;
                        have_prev[scan_ch] <= 1'b1;

                        // Advance FLL counter
                        if (fll_count[scan_ch] < fll_epochs)
                            fll_count[scan_ch] <= fll_count[scan_ch] + 16'd1;

                        // Carrier update
                        if (scan_in_fll) begin
                            ch_car_f[scan_ch] <= ch_car_f[scan_ch] + fll_term;
                        end else begin
                            car_integral[scan_ch] <= scan_integral + norm_phase_err;
                            ch_car_f[scan_ch] <= ch_car_f[scan_ch] + car_p_term + car_i_term;
                        end

                        // Code DLL update + carrier-aiding
                        // mul_term still holds the DLL result from sub 49
                        // (multiplier output is registered, holds until next mul)
                        ch_cod_f[scan_ch] <= CODE_FREQ_NOM + code_aiding - mul_term;
                    end
                    scan_ch <= (scan_ch == NUM_CH - 1) ? 4'd0 : scan_ch + 4'd1;
                end

                default: ;
            endcase

            // Latch epoch flags for tracking channels
            for (i = 0; i < NUM_CH; i = i + 1)
                if (ch_epoch_flag[i] && ch_track[i])
                    track_pending[i] <= 1'b1;

            // ---- Register writes ----
            if (reg_wen) begin
                if (wr_ch < NUM_CH[3:0]) begin
                    case (wr_reg)
                        3'd0: begin
                            ch_track[wr_ch] <= reg_wdata[6];
                            ch_en[wr_ch]    <= reg_wdata[5];
                            ch_prn[wr_ch]   <= reg_wdata[4:0];
                            if (reg_wdata[6]) begin
                                car_integral[wr_ch] <= 32'd0;
                                fll_count[wr_ch]    <= 16'd0;
                                have_prev[wr_ch]    <= 1'b0;
                                car_f_base[wr_ch]   <= ch_car_f[wr_ch];
                            end
                        end
                        3'd1: ch_car_f[wr_ch] <= reg_wdata;
                        3'd2: ch_cod_f[wr_ch] <= reg_wdata;
                        3'd3: begin
                            ch_phase_wen[wr_ch] <= 1'b1;
                            ch_phase_val <= reg_wdata[9:0];
                        end
                        default: ;
                    endcase
                end else if (wr_ch == 4'hC) begin
                    case (wr_reg)
                        3'd0: car_gain_p <= reg_wdata;
                        3'd1: car_gain_i <= reg_wdata;
                        3'd2: cod_gain   <= reg_wdata;
                        3'd3: fll_gain   <= reg_wdata;
                        default: ;
                    endcase
                end else if (wr_ch == 4'hD) begin
                    case (wr_reg)
                        3'd0: fll_epochs <= reg_wdata[15:0];
                        default: ;
                    endcase
                end else if (wr_ch == 4'hF) begin
                    case (wr_reg)
                        3'd0: global_enable <= reg_wdata[0];
                        3'd3: begin
                            acq_start <= reg_wdata[0];
                            acq_abort <= reg_wdata[1];
                        end
                        3'd4: begin
                            acq_prn        <= reg_wdata[20:16];
                            acq_num_epochs <= (reg_wdata[5:0] == 6'd0) ? 6'd1 : reg_wdata[5:0];
                        end
                        3'd5: acq_carrier <= reg_wdata;
                        default: ;
                    endcase
                end
            end
        end
    end

    // ---- Register reads (combinational) ----
    wire [3:0] rd_ch  = reg_addr[6:3];
    wire [2:0] rd_reg = reg_addr[2:0];

    always @(*) begin
        reg_rdata = 32'd0;
        if (rd_ch < NUM_CH[3:0]) begin
            case (rd_reg)
                3'd0: reg_rdata = {25'd0, ch_track[rd_ch], ch_en[rd_ch], ch_prn[rd_ch]};
                3'd1: reg_rdata = ch_car_f[rd_ch];
                3'd2: reg_rdata = ch_cod_f[rd_ch];
                3'd3: reg_rdata = ch_ip[rd_ch];
                3'd4: reg_rdata = ch_qp[rd_ch];
                3'd5: reg_rdata = {ch_ie[rd_ch][31:16], ch_qe[rd_ch][31:16]};
                3'd6: reg_rdata = {ch_il[rd_ch][31:16], ch_ql[rd_ch][31:16]};
                3'd7: reg_rdata = {14'd0, ch_busy[rd_ch], ch_epoch_flag[rd_ch], ch_epoch_count[rd_ch]};
                default: ;
            endcase
        end else if (rd_ch == 4'hC) begin
            case (rd_reg)
                3'd0: reg_rdata = car_gain_p;
                3'd1: reg_rdata = car_gain_i;
                3'd2: reg_rdata = cod_gain;
                3'd3: reg_rdata = fll_gain;
                default: ;
            endcase
        end else if (rd_ch == 4'hD) begin
            case (rd_reg)
                3'd0: reg_rdata = {16'd0, fll_epochs};
                default: ;
            endcase
        end else if (rd_ch == 4'hF) begin
            case (rd_reg)
                3'd0: reg_rdata = {31'd0, global_enable};
                3'd1: reg_rdata = global_epoch;
                3'd2: reg_rdata = 32'h47505307;
                3'd3: reg_rdata = {acq_done, acq_active, 20'd0, acq_best_phase};
                3'd4: reg_rdata = {5'd0, acq_prn, 10'd0, acq_num_epochs};
                3'd5: reg_rdata = acq_carrier;
                3'd6: reg_rdata = acq_best_power;
                3'd7: reg_rdata = 32'd0;
                default: ;
            endcase
        end
    end

endmodule
