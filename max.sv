`timescale 1ps / 1ps

module max
#(
    parameter D_W = 32
)
(
    input  logic                  clk,
    input  logic                  rst,
    input  logic                  enable,
    input  logic                  initialize,
    input  logic signed [D_W-1:0] in_data,
    output logic signed [D_W-1:0] result
);

logic signed [D_W-1:0] maximum;

always_ff @(posedge clk) begin
    if(rst) begin
        result <= 0;
    end else begin
        if (enable) begin
            result <= maximum;
        end
    end
end

always_comb begin
    if(initialize) begin
        maximum = in_data;
    end else begin
        maximum = (result > in_data) ? result : in_data;
    end
end

endmodule
