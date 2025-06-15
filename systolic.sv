`timescale 1ps / 1ps

module systolic
#(
    parameter integer D_W     = 8,      // operand data width
    parameter integer D_W_ACC = 32,     // accumulator data width
    parameter integer N1      = 8,
    parameter integer N2      = 4
)
(
    input  logic                      clk,
    input  logic                      rst,
    input  logic        [N2-1:0]      init [N1-1:0],
    input  logic signed [D_W-1:0]     A    [N1-1:0],
    input  logic signed [D_W-1:0]     B    [N2-1:0],
    output logic signed [D_W_ACC-1:0] D    [N1-1:0],
    output logic        [N1-1:0]      valid_D
);

logic signed [D_W-1:0]      pe_a        [N1-1:0][N2-1:0]; //intermediate wires to store the result of pe modules horizontally
logic signed [D_W-1:0]      pe_b        [N1-1:0][N2-1:0]; //intermediate wires to store the result of pe modules vertically
logic signed [D_W_ACC-1:0]  pe_out      [N1-1:0][N2-1:0];
logic                       pe_valid    [N1-1:0][N2-1:0];

genvar i, j;

generate
    for (i = 0; i < N1; i = i + 1) begin : rows
        for (j = 0; j < N2; j = j + 1) begin : columns
            pe #(
                .D_W(D_W),
                .D_W_ACC(D_W_ACC)
            )
            pe_module(
                .clk(clk),
                .rst(rst),
                .init(init[i][j]),
                .in_a((j == 0) ? A[i] : pe_a[i][j-1]), //shifting right horizontally, first columns are fed by A, last columns are not set
                .in_b((i == 0) ? B[j] : pe_b[i-1][j]), //shifting down vertically, first rows are fed by B, last rows are not set
                .in_data((j == 0) ? 0 : pe_out[i][j-1]), //written to
                .in_valid((j == 0) ? 0 : pe_valid[i][j-1]), //written to
                .out_a(pe_a[i][j]),
                .out_b(pe_b[i][j]),
                .out_data(pe_out[i][j]),
                .out_valid(pe_valid[i][j])
            );
        end
    end
endgenerate

// always_ff @(posedge clk) begin
//     if (rst) begin
//         valid_D <= 0;
//         for (int row = 0; row < N1; row = row + 1) begin
//             D[row] <= 0;
//         end
//     end else begin
//         for (int row = 0; row < N1; row = row + 1) begin
//             D[:] <= pe_out[row][N2-1];
//             valid_D[row] <= pe_valid[row][N2-1];
//         end
//     end
// end

always_comb begin
    for (int row = 0; row < N1; row = row + 1) begin
        D[row] = pe_out[row][N2-1];
        valid_D[row] = pe_valid[row][N2-1];
    end
end

endmodule
