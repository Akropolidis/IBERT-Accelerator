`timescale 1ps / 1ps

module pe
#(
    parameter integer D_W     = 8,      // operand data width
    parameter integer D_W_ACC = 32      // accumulator data width
)
(
    input  logic                      clk,
    input  logic                      rst,
    input  logic                      init,
    input  logic signed [D_W-1:0]     in_a,
    input  logic signed [D_W-1:0]     in_b,
    input  logic signed [D_W_ACC-1:0] in_data,
    input  logic                      in_valid,
    output logic  signed [D_W-1:0]     out_a,
    output logic  signed [D_W-1:0]     out_b,
    output logic  signed [D_W_ACC-1:0] out_data,
    output logic                       out_valid
);

logic signed [D_W_ACC-1:0] out_data_r;
logic signed [D_W_ACC-1:0] in_data_r;
logic in_valid_r;

logic signed [D_W_ACC-1:0] mult_and_acc;

always_ff @(posedge clk) begin
    if(rst) begin
        out_a <= 0;
        out_b <= 0;
        out_data <= 0;
        out_valid <= 0;

        out_data_r <= 0;
        in_data_r <= 0;
        in_valid_r <= 0;

        mult_and_acc <= 0;
    end else begin
        out_a <= in_a;
        out_b <= in_b;
        
        if (init) begin
            mult_and_acc <= in_a * in_b;
        end else begin
            mult_and_acc <= mult_and_acc + in_a * in_b;
        end

        in_data_r <= in_data;
        in_valid_r <= in_valid;

        if (in_valid) begin
            out_data_r <= in_data;
        end

        if (!in_valid_r && !init) begin
            out_data  <= in_data; //Default value
            out_valid <= 0;
        end else if (init) begin
            out_data  <= mult_and_acc; //MAC case
            out_valid <= init;
        end else begin
            out_data  <= out_data_r; //in_valid case where values are delayed by 1 cycle
            out_valid <= in_valid_r;
        end
    end
end

endmodule
