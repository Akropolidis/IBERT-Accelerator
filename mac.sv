`timescale 1ps / 1ps

module mac
#(
    parameter D_W     = 32,
    parameter D_W_ACC = 32
)
(
    input logic                      clk,
    input logic                      rst,
    input logic                      enable,
    input logic                      initialize,
    input logic signed [D_W-1:0]     a,
    input logic signed [D_W-1:0]     b,
    output logic signed [D_W_ACC-1:0] result
);

logic signed [D_W_ACC-1:0] mult_and_acc;

always_ff @(posedge clk) begin
    if(rst) begin
        result <= 0;
    end else begin
        if (enable) begin
            result <= mult_and_acc;
        end
    end
end

always_comb begin
    if(initialize) begin
        mult_and_acc = a * b;
    end else begin
        mult_and_acc = result + a * b;
    end
end

endmodule
