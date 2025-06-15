`timescale 1ps / 1ps

module acc
#(
    parameter D_W     = 32,
    parameter D_W_ACC = 32
)
(
    input  logic                      clk,
    input  logic                      rst,
    input  logic                      enable,
    input  logic                      initialize,
    input  logic signed [D_W-1:0]     in_data,
    output logic signed [D_W_ACC-1:0] result
);

logic signed [D_W_ACC-1:0] accumulated;

always_ff @(posedge clk) begin
    if(rst) begin
        result <= 0;
    end else begin
        if (enable) begin
            result <= accumulated;
        end
    end
end

always_comb begin
    if(initialize) begin
        accumulated = in_data;
    end else begin
        accumulated = result + in_data;
    end
end

endmodule
