`timescale 1ns/1ps
// Learning example: discard four LSBs of one signed sample component.
// One-entry output register; upstream must hold data while valid && !ready.
// This is not yet connected to the LimeSDR gateware or its sample format.
module quantizer (
    input wire clk,
    input wire reset,
    input wire signed [11:0] in_data,
    input wire in_valid,
    output wire in_ready,
    output reg signed [7:0] out_data,
    output reg out_valid,
    input wire out_ready
);
    assign in_ready = !reset && (!out_valid || out_ready);
    always @(posedge clk) begin
        if (reset) begin
            out_valid <= 0;
            out_data <= 0;
        end else if (in_ready) begin
            out_valid <= in_valid;
            if (in_valid)
                out_data <= in_data[11:4];
        end
    end
endmodule
