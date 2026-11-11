// GPS correlator top-level — SPI slave + 12-channel correlator
//
// Integration point in LimeSDR gateware:
//   i_in/q_in/sample_valid: tap from RXPathTop's I/Q output
//   spi_*: connect to FPGA_GPIO pins on J3 header
//
// Pin assignment (suggested, matches J3 on LimeSDR Mini v2):
//   spi_sck  → FPGA_GPIO[0]
//   spi_mosi → FPGA_GPIO[1]
//   spi_miso → FPGA_GPIO[2]
//   spi_cs_n → FPGA_GPIO[3]

module gps_top (
    input  wire        clk,
    input  wire        rst,

    // I/Q samples from LMS7002M (12-bit signed, at sample rate)
    input  wire signed [11:0] i_in,
    input  wire signed [11:0] q_in,
    input  wire        sample_valid,

    // SPI interface to flight computer
    input  wire        spi_sck,
    input  wire        spi_mosi,
    output wire        spi_miso,
    input  wire        spi_cs_n
);

    wire [6:0]  reg_addr;
    wire [31:0] reg_wdata;
    wire [31:0] reg_rdata;
    wire        reg_wen;

    gps_spi_slave spi (
        .clk       (clk),
        .rst       (rst),
        .spi_sck   (spi_sck),
        .spi_mosi  (spi_mosi),
        .spi_miso  (spi_miso),
        .spi_cs_n  (spi_cs_n),
        .reg_addr  (reg_addr),
        .reg_wdata (reg_wdata),
        .reg_rdata (reg_rdata),
        .reg_wen   (reg_wen)
    );

    gps_correlator corr (
        .clk          (clk),
        .rst          (rst),
        .i_in         (i_in),
        .q_in         (q_in),
        .sample_valid (sample_valid),
        .reg_addr     (reg_addr),
        .reg_wdata    (reg_wdata),
        .reg_rdata    (reg_rdata),
        .reg_wen      (reg_wen)
    );

endmodule
