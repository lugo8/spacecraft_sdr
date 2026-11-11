// GPS C/A code generator — Gold code from two 10-stage LFSRs (IS-GPS-200)
module gps_ca_gen (
    input  wire        clk,
    input  wire        rst,
    input  wire [4:0]  prn,      // PRN 1-32 (0 = disabled)
    input  wire        chip_en,  // advance one chip
    input  wire        phase_wen,// preset chip_cnt to phase_in
    input  wire [9:0]  phase_in, // chip phase preset (0-1022)
    output wire        code_out, // current C/A chip (1/0 = +1/-1)
    output wire        epoch,    // pulse on last chip of 1023-chip period
    output wire        busy      // high while seeking to preset phase
);

    reg [10:1] g1, g2;
    reg [9:0]  chip_cnt;

    // G1: x^10 + x^3 + 1
    wire g1_fb = g1[10] ^ g1[3];

    // G2: x^10 + x^9 + x^8 + x^6 + x^3 + x^2 + 1
    wire g2_fb = g2[10] ^ g2[9] ^ g2[8] ^ g2[6] ^ g2[3] ^ g2[2];

    // PRN-specific G2 output tap selection (IS-GPS-200 Table 3-Ia)
    reg g2_sel;
    always @(*) begin
        case (prn)
            5'd1:    g2_sel = g2[2]  ^ g2[6];
            5'd2:    g2_sel = g2[3]  ^ g2[7];
            5'd3:    g2_sel = g2[4]  ^ g2[8];
            5'd4:    g2_sel = g2[5]  ^ g2[9];
            5'd5:    g2_sel = g2[1]  ^ g2[9];
            5'd6:    g2_sel = g2[2]  ^ g2[10];
            5'd7:    g2_sel = g2[1]  ^ g2[8];
            5'd8:    g2_sel = g2[2]  ^ g2[9];
            5'd9:    g2_sel = g2[3]  ^ g2[10];
            5'd10:   g2_sel = g2[2]  ^ g2[3];
            5'd11:   g2_sel = g2[3]  ^ g2[4];
            5'd12:   g2_sel = g2[5]  ^ g2[6];
            5'd13:   g2_sel = g2[6]  ^ g2[7];
            5'd14:   g2_sel = g2[7]  ^ g2[8];
            5'd15:   g2_sel = g2[8]  ^ g2[9];
            5'd16:   g2_sel = g2[9]  ^ g2[10];
            5'd17:   g2_sel = g2[1]  ^ g2[4];
            5'd18:   g2_sel = g2[2]  ^ g2[5];
            5'd19:   g2_sel = g2[3]  ^ g2[6];
            5'd20:   g2_sel = g2[4]  ^ g2[7];
            5'd21:   g2_sel = g2[5]  ^ g2[8];
            5'd22:   g2_sel = g2[6]  ^ g2[9];
            5'd23:   g2_sel = g2[1]  ^ g2[3];
            5'd24:   g2_sel = g2[4]  ^ g2[6];
            5'd25:   g2_sel = g2[5]  ^ g2[7];
            5'd26:   g2_sel = g2[6]  ^ g2[8];
            5'd27:   g2_sel = g2[7]  ^ g2[9];
            5'd28:   g2_sel = g2[8]  ^ g2[10];
            5'd29:   g2_sel = g2[1]  ^ g2[6];
            5'd30:   g2_sel = g2[2]  ^ g2[7];
            5'd31:   g2_sel = g2[3]  ^ g2[8];
            5'd32:   g2_sel = g2[4]  ^ g2[9];
            default: g2_sel = 1'b0;
        endcase
    end

    assign code_out = g1[10] ^ g2_sel;
    assign epoch = chip_en & (chip_cnt == 10'd1022) & ~seeking;

    // Phase preset: reset LFSRs then advance chip-by-chip to target phase.
    // Takes up to 1023 clocks (~25 us at 40 MHz).
    reg        seeking;
    reg [9:0]  seek_target;
    assign busy = seeking;

    always @(posedge clk) begin
        if (rst || prn == 5'd0) begin
            g1          <= 10'h3FF;
            g2          <= 10'h3FF;
            chip_cnt    <= 10'd0;
            seeking     <= 1'b0;
            seek_target <= 10'd0;
        end else if (phase_wen) begin
            g1          <= 10'h3FF;
            g2          <= 10'h3FF;
            chip_cnt    <= 10'd0;
            seeking     <= (phase_in != 10'd0);
            seek_target <= phase_in;
        end else if (seeking) begin
            g1       <= {g1[9:1], g1_fb};
            g2       <= {g2[9:1], g2_fb};
            chip_cnt <= chip_cnt + 10'd1;
            if (chip_cnt + 10'd1 == seek_target)
                seeking <= 1'b0;
        end else if (chip_en) begin
            if (chip_cnt == 10'd1022) begin
                g1       <= 10'h3FF;
                g2       <= 10'h3FF;
                chip_cnt <= 10'd0;
            end else begin
                g1       <= {g1[9:1], g1_fb};
                g2       <= {g2[9:1], g2_fb};
                chip_cnt <= chip_cnt + 10'd1;
            end
        end
    end

endmodule
