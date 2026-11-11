// 12-channel GPS L1 C/A correlator with register interface
//
// Register map (7-bit address):
//   addr[6:3] = channel (0-11), addr[2:0] = register
//   Channel 15 (addr[6:3] = 4'hF) = global registers
//
// Per-channel registers:
//   0: CTRL       [RW]  {26'b0, enable, prn[4:0]}
//   1: CAR_FREQ   [RW]  carrier NCO frequency word
//   2: COD_FREQ   [RW]  code NCO frequency word
//   3: COD_PHASE  [W]   {22'b0, chip_phase[9:0]} — preset code phase (0-1022)
//   3: IP         [R]   I-Prompt accumulator (32-bit signed)
//   4: QP         [R]   Q-Prompt accumulator (32-bit signed)
//   5: IE_QE      [R]   {ie[31:16], qe[31:16]} upper halves packed
//   6: IL_QL      [R]   {il[31:16], ql[31:16]} upper halves packed
//   7: STATUS     [R]   {14'b0, busy, epoch_flag, epoch_count[15:0]}
//
// Global registers (channel 15):
//   0: GLOB_CTRL  [RW]  {31'b0, enable}
//   1: EPOCH_CNT  [R]   global epoch counter
//   2: VERSION    [R]   0x47505301 ("GPS" + v1)
//
// Carrier NCO: freq_word = round(f_Hz / f_sample * 2^32)
//   e.g. 27500 Hz at 4.096 MS/s -> 0x01B80000
// Code NCO:    freq_word = round(1.023e6 / f_sample * 2^32)
//   Nominal 1.023 MHz at 4.096 MS/s -> 0x3FF00000

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

    localparam NUM_CH = 12;
    localparam CODE_FREQ_NOM = 32'h3FF00000;

    // ---- Global control ----
    reg global_enable;
    reg [31:0] global_epoch;

    // ---- Per-channel config ----
    reg [4:0]  ch_prn    [0:NUM_CH-1];
    reg        ch_en     [0:NUM_CH-1];
    reg [31:0] ch_car_f  [0:NUM_CH-1];
    reg [31:0] ch_cod_f  [0:NUM_CH-1];

    // Phase preset (active for one clock per write)
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

    // ---- Register writes ----
    wire [3:0] wr_ch  = reg_addr[6:3];
    wire [2:0] wr_reg = reg_addr[2:0];

    integer i;
    always @(posedge clk) begin
        ch_phase_wen <= {NUM_CH{1'b0}};

        if (rst) begin
            global_enable <= 1'b0;
            global_epoch  <= 32'd0;
            for (i = 0; i < NUM_CH; i = i + 1) begin
                ch_prn[i]   <= 5'd0;
                ch_en[i]    <= 1'b0;
                ch_car_f[i] <= 32'd0;
                ch_cod_f[i] <= CODE_FREQ_NOM;
            end
        end else begin
            if (ch_epoch_flag[0])
                global_epoch <= global_epoch + 32'd1;

            if (reg_wen) begin
                if (wr_ch < NUM_CH[3:0]) begin
                    case (wr_reg)
                        3'd0: begin
                            ch_en[wr_ch]  <= reg_wdata[5];
                            ch_prn[wr_ch] <= reg_wdata[4:0];
                        end
                        3'd1: ch_car_f[wr_ch] <= reg_wdata;
                        3'd2: ch_cod_f[wr_ch] <= reg_wdata;
                        3'd3: begin
                            ch_phase_wen[wr_ch] <= 1'b1;
                            ch_phase_val <= reg_wdata[9:0];
                        end
                        default: ;
                    endcase
                end else if (wr_ch == 4'hF) begin
                    if (wr_reg == 3'd0)
                        global_enable <= reg_wdata[0];
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
                3'd0: reg_rdata = {26'd0, ch_en[rd_ch], ch_prn[rd_ch]};
                3'd1: reg_rdata = ch_car_f[rd_ch];
                3'd2: reg_rdata = ch_cod_f[rd_ch];
                3'd3: reg_rdata = ch_ip[rd_ch];
                3'd4: reg_rdata = ch_qp[rd_ch];
                3'd5: reg_rdata = {ch_ie[rd_ch][31:16], ch_qe[rd_ch][31:16]};
                3'd6: reg_rdata = {ch_il[rd_ch][31:16], ch_ql[rd_ch][31:16]};
                3'd7: reg_rdata = {14'd0, ch_busy[rd_ch], ch_epoch_flag[rd_ch], ch_epoch_count[rd_ch]};
                default: ;
            endcase
        end else if (rd_ch == 4'hF) begin
            case (rd_reg)
                3'd0: reg_rdata = {31'd0, global_enable};
                3'd1: reg_rdata = global_epoch;
                3'd2: reg_rdata = 32'h47505301;
                default: ;
            endcase
        end
    end

endmodule
