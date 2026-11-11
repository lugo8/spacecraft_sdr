// SPI slave for GPS correlator register access
//
// Protocol (CPOL=0, CPHA=0):
//   CS low → 8-bit address → 32-bit data → CS high
//   Address byte: {W_n_R, addr[6:0]}
//     W_n_R = 1: write (MOSI carries 32-bit write data)
//     W_n_R = 0: read  (MISO returns 32-bit register data)
//   Total: 40 SCK edges per transaction
//
// All SPI signals are synchronized to clk internally.

module gps_spi_slave (
    input  wire        clk,
    input  wire        rst,

    input  wire        spi_sck,
    input  wire        spi_mosi,
    output wire        spi_miso,
    input  wire        spi_cs_n,

    output reg  [6:0]  reg_addr,
    output reg  [31:0] reg_wdata,
    input  wire [31:0] reg_rdata,
    output reg         reg_wen
);

    // ---- Synchronize SPI signals (3-stage for metastability) ----
    reg [2:0] sck_sr, cs_sr;
    reg [1:0] mosi_sr;

    always @(posedge clk) begin
        sck_sr  <= {sck_sr[1:0],  spi_sck};
        cs_sr   <= {cs_sr[1:0],   spi_cs_n};
        mosi_sr <= {mosi_sr[0],   spi_mosi};
    end

    wire sck_rise = (sck_sr[2:1] == 2'b01);
    wire sck_fall = (sck_sr[2:1] == 2'b10);
    wire cs_n     = cs_sr[2];
    wire mosi     = mosi_sr[1];

    // ---- State ----
    reg [5:0]  bit_cnt;
    reg [7:0]  addr_sr;
    reg [31:0] tx_sr;
    reg [31:0] rx_sr;
    reg        is_write;
    reg        addr_done;
    reg        tx_loaded;

    assign spi_miso = tx_sr[31];

    always @(posedge clk) begin
        reg_wen <= 1'b0;

        if (rst || cs_n) begin
            bit_cnt   <= 6'd0;
            addr_sr   <= 8'd0;
            tx_sr     <= 32'd0;
            rx_sr     <= 32'd0;
            is_write  <= 1'b0;
            addr_done <= 1'b0;
            tx_loaded <= 1'b0;
        end else begin
            // ---- MOSI: sample on SCK rising edge ----
            if (sck_rise) begin
                bit_cnt <= bit_cnt + 6'd1;

                if (!addr_done) begin
                    addr_sr <= {addr_sr[6:0], mosi};
                    if (bit_cnt == 6'd7) begin
                        addr_done <= 1'b1;
                        is_write  <= addr_sr[6];
                        reg_addr  <= {addr_sr[5:0], mosi};
                    end
                end else begin
                    rx_sr <= {rx_sr[30:0], mosi};
                end
            end

            // ---- Load TX shift register for reads ----
            if (addr_done && !tx_loaded && !is_write) begin
                tx_sr     <= reg_rdata;
                tx_loaded <= 1'b1;
            end

            // ---- MISO: shift out on SCK falling edge ----
            if (sck_fall && addr_done && tx_loaded) begin
                tx_sr <= {tx_sr[30:0], 1'b0};
            end

            // ---- Write: complete after 40 bits ----
            if (sck_rise && addr_done && is_write && bit_cnt == 6'd39) begin
                reg_wdata <= {rx_sr[30:0], mosi};
                reg_wen   <= 1'b1;
            end
        end
    end

endmodule
