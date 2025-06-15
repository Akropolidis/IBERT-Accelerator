
`timescale 1ps / 1ps
import dims::*;

module mm_ln
#(
    parameter integer D_W               = 8,
    parameter integer D_W_ACC           = 32,
    parameter integer LN_BITS           = 22,
    parameter integer N1                = 4,
    parameter integer N2                = 4,
    parameter integer MATRIXSIZE_W      = 24,
    parameter         BLOCKED_A         = 0,
    parameter integer MEM_DEPTH_A       = 6144,
    parameter integer MEM_DEPTH_B       = 147456,
    parameter integer MEM_DEPTH_D       = 6144,
    parameter integer MAT_ADD_MEM_DEPTH = 6144,
    parameter integer REQ_MEM_DEPTH     = 768
)
(
    input wire                      clk,
    input wire                      fclk,
    input wire                      rst,

    axi_stream_if.axi_in            in_X,

    axi_stream_if.axi_in            in_W,
    axi_stream_if.axi_in            in_W_bias,
    axi_stream_if.axi_in            in_W_m,
    axi_stream_if.axi_in            in_W_e,

    axi_stream_if.axi_in            in_R,
    axi_stream_if.axi_in            in_R_m,
    axi_stream_if.axi_in            in_R_e,

    axi_stream_if.axi_in            in_ln_bias,

    axi_stream_if.axi_in            in_out_m,
    axi_stream_if.axi_in            in_out_e,

    axi_stream_if.axi_out           mm_ln_out,

    input  dimensions               mm_dimensions
);

axi_stream_if #(D_W_ACC) Y        ();
axi_stream_if #(D_W_ACC) W_bias   ();
axi_stream_if #(D_W_ACC) W_m      ();
axi_stream_if #(D_W)     W_e      ();
axi_stream_if #(D_W_ACC) Y_req    ();
axi_stream_if #(D_W_ACC) R        ();
axi_stream_if #(D_W_ACC) R_m      ();
axi_stream_if #(D_W)     R_e      ();
axi_stream_if #(D_W_ACC) R_req    ();
axi_stream_if #(D_W_ACC) R_bias   ();
axi_stream_if #(D_W_ACC) Z        ();
axi_stream_if #(D_W_ACC) ln_out   ();
axi_stream_if #(D_W_ACC) out_bias ();
axi_stream_if #(D_W_ACC) out_m    ();
axi_stream_if #(D_W)     out_e    ();

assign R_bias.tdata = {D_W_ACC{1'b0}};
assign R_bias.tlast = 1'b1;
assign R_bias.tvalid = 1'b1;

assign out_bias.tdata = {D_W_ACC{1'b0}};
assign out_bias.tlast = 1'b1;
assign out_bias.tvalid = 1'b1;

mm_pp_blocked_A #(
    .D_W(D_W),
    .D_W_ACC(D_W_ACC),
    .N1(N1),
    .N2(N2),
    .MATRIXSIZE_W(MATRIXSIZE_W),
    .MEM_DEPTH_A(MEM_DEPTH_A),
    .MEM_DEPTH_B(MEM_DEPTH_B),
    .MEM_DEPTH_D(MEM_DEPTH_D),
    .TRANSPOSE_B(0)
)
mm_Y (
    .mm_clk(clk),
    .mm_fclk(fclk),
    .mm_rst_n(~rst),

    .in_A(in_X),
    .in_B(in_W),

    .out_D(Y),

    .BLOCKS(mm_dimensions.BLOCKS),
    .BLOCK_WIDTH(mm_dimensions.BLOCK_WIDTH),
    .BLOCK_WIDTHdN2(mm_dimensions.BLOCK_WIDTHdN2),
    .BLOCK_SIZEdN2(mm_dimensions.BLOCK_SIZEdN2),
    .M1xBLOCK_WIDTHdN1xN2(mm_dimensions.M1xBLOCK_WIDTHdN1xN2),
    .M1xBLOCK_WIDTHdN1(mm_dimensions.M1xBLOCK_WIDTHdN1),
    .BLOCKS_A(mm_dimensions.BLOCKS_A),
    .BLOCK_WIDTH_A(mm_dimensions.BLOCK_WIDTH_A),
    .M2(mm_dimensions.M2),
    .M3dN2(mm_dimensions.M3dN2),
    .M1xM3dN1(mm_dimensions.M1xM3dN1),
    .M1dN1(mm_dimensions.M1dN1)
);

stream_vector_mem #(
    .D_W(D_W_ACC),
    .MATRIXSIZE_W(MATRIXSIZE_W),
    .MEM_DEPTH(REQ_MEM_DEPTH)
)
vec_mem_W_bias (
    .clk(clk),
    .rst(rst),

    .in_vec(in_W_bias),
    .out_vec(W_bias),

    .DIM1(mm_dimensions.M1),
    .DIM2(mm_dimensions.M3)
);

stream_vector_mem #(
    .D_W(D_W_ACC),
    .MATRIXSIZE_W(MATRIXSIZE_W),
    .MEM_DEPTH(REQ_MEM_DEPTH)
)
vec_mem_W_m (
    .clk(clk),
    .rst(rst),

    .in_vec(in_W_m),
    .out_vec(W_m),

    .DIM1(mm_dimensions.M1),
    .DIM2(mm_dimensions.M3)
);

stream_vector_mem #(
    .D_W(D_W),
    .MATRIXSIZE_W(MATRIXSIZE_W),
    .MEM_DEPTH(REQ_MEM_DEPTH)
)
vec_mem_W_e (
    .clk(clk),
    .rst(rst),

    .in_vec(in_W_e),
    .out_vec(W_e),

    .DIM1(mm_dimensions.M1),
    .DIM2(mm_dimensions.M3)
);

requant #(
    .D_W(D_W),
    .D_W_ACC(D_W_ACC),
    .OUT_BITS(D_W_ACC),
    .CLIP(0)
)
requant_Y (
    .clk(clk),
    .rst(rst),

    .in_A(Y),
    .in_bias(W_bias),
    .in_m(W_m),
    .in_e(W_e),

    .out_D(Y_req)
);

stream_matrix_mem #(
    .D_W(D_W),
    .MATRIXSIZE_W(MATRIXSIZE_W),
    .MEM_DEPTH(MAT_ADD_MEM_DEPTH)
)
mat_mem_R (
    .clk(clk),
    .rst(rst),

    .in_mat(in_R),
    .out_mat(R),

    .DEPTH(mm_dimensions.M1xM3)
);

stream_scalar_mem #(
    .D_W(D_W_ACC),
    .MATRIXSIZE_W(MATRIXSIZE_W)
)
sca_mem_R_m (
    .clk(clk),
    .rst(rst),

    .in_sca(in_R_m),
    .out_sca(R_m),

    .DIM1(mm_dimensions.M1),
    .DIM2(mm_dimensions.M3)
);

stream_scalar_mem #(
    .D_W(D_W),
    .MATRIXSIZE_W(MATRIXSIZE_W)
)
sca_mem_R_e (
    .clk(clk),
    .rst(rst),

    .in_sca(in_R_e),
    .out_sca(R_e),

    .DIM1(mm_dimensions.M1),
    .DIM2(mm_dimensions.M3)
);

requant #(
    .D_W(D_W),
    .D_W_ACC(D_W_ACC),
    .OUT_BITS(D_W_ACC),
    .CLIP(0)
)
requant_R (
    .clk(clk),
    .rst(rst),

    .in_A(R),
    .in_bias(R_bias),
    .in_m(R_m),
    .in_e(R_e),

    .out_D(R_req)
);

mat_add #(
    .D_W(D_W_ACC),
    .MAX_BITS(LN_BITS)
)
mat_add_Z (
    .clk(clk),
    .rst(rst),

    .in_R(R_req),
    .in_Y(Y_req),

    .out_Z(Z)
);

layer_norm_top #(
    .D_W(D_W),
    .D_W_ACC(D_W_ACC),
    .MATRIXSIZE_W(MATRIXSIZE_W)
)
layer_norm_top_unit (
    .clk(clk),
    .rst(rst),

    .in_ln_bias(in_ln_bias),
    .qin(Z),
    .qout(ln_out),

    .DIM1(mm_dimensions.M1),
    .DIM2(mm_dimensions.M3)
);

stream_vector_mem #(
    .D_W(D_W_ACC),
    .MATRIXSIZE_W(MATRIXSIZE_W),
    .MEM_DEPTH(REQ_MEM_DEPTH)
)
vec_mem_out_m (
    .clk(clk),
    .rst(rst),

    .in_vec(in_out_m),
    .out_vec(out_m),

    .DIM1(mm_dimensions.M1),
    .DIM2(mm_dimensions.M3)
);

stream_vector_mem #(
    .D_W(D_W),
    .MATRIXSIZE_W(MATRIXSIZE_W),
    .MEM_DEPTH(REQ_MEM_DEPTH)
)
vec_mem_out_e (
    .clk(clk),
    .rst(rst),

    .in_vec(in_out_e),
    .out_vec(out_e),

    .DIM1(mm_dimensions.M1),
    .DIM2(mm_dimensions.M3)
);

requant #(
    .D_W(D_W),
    .D_W_ACC(D_W_ACC),
    .OUT_BITS(D_W),
    .CLIP(1)
)
requant_out (
    .clk(clk),
    .rst(rst),

    .in_A(ln_out),
    .in_bias(out_bias),
    .in_m(out_m),
    .in_e(out_e),

    .out_D(mm_ln_out)
);

endmodule
