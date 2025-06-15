
`timescale 1ps / 1ps
import dims::*;

module attn_head
#(
    parameter integer D_W           = 8,
    parameter integer D_W_ACC       = 32,
    parameter integer N1            = 4,
    parameter integer N2            = 4,
    parameter integer sN1           = 4,
    parameter integer sN2           = 4,
    parameter integer LAYERS        = 12,
    parameter integer SOFTMAX_N     = 32,
    parameter integer MATRIXSIZE_W  = 24,
    parameter integer KEEP_A        = 1,
    parameter integer MEM_DEPTH_A   = 4096,
    parameter integer MEM_DEPTH_B   = 4096,
    parameter integer MEM_DEPTH_D   = 4096,
    parameter integer MEM_DEPTH_Q   = 4096,
    parameter integer MEM_DEPTH_K   = 4096,
    parameter integer MEM_DEPTH_S   = 4096,
    parameter integer MEM_DEPTH_P   = MEM_DEPTH_S,
    parameter integer MEM_DEPTH_V   = MEM_DEPTH_K,
    parameter integer MEM_DEPTH_C   = MEM_DEPTH_Q,
    parameter integer REQ_MEM_DEPTH = 64
)
(
    input wire                      clk,
    input wire                      fclk,
    input wire                      rst,
    input wire [$clog2(LAYERS)-1:0] layer,

    axi_stream_if.axi_in            in_I,

    axi_stream_if.axi_in            in_W_Q,
    axi_stream_if.axi_in            in_W_K,
    axi_stream_if.axi_in            in_W_V,

    axi_stream_if.axi_in            in_bias_Q,
    axi_stream_if.axi_in            in_bias_K,
    axi_stream_if.axi_in            in_bias_V,

    axi_stream_if.axi_in            in_m_Q,
    axi_stream_if.axi_in            in_m_K,
    axi_stream_if.axi_in            in_m_V,

    axi_stream_if.axi_in            in_e_Q,
    axi_stream_if.axi_in            in_e_K,
    axi_stream_if.axi_in            in_e_V,

    axi_stream_if.axi_in            in_m_C,
    axi_stream_if.axi_in            in_e_C,

    axi_stream_if.axi_out           attn_head_out,

    input dimensions                mm_dimensions
);

axi_stream_if             W           ();
axi_stream_if #(D_W_ACC)  QKV         ();
axi_stream_if             bias        ();
axi_stream_if             m           ();
axi_stream_if             e           ();
axi_stream_if #(D_W_ACC)  bias_QKV    ();
axi_stream_if #(D_W_ACC)  m_QKV       ();
axi_stream_if #(D_W)      e_QKV       ();
axi_stream_if #(D_W)      QKV_req     ();
axi_stream_if #(D_W)      Q_req       ();
axi_stream_if #(D_W)      K_req       ();
axi_stream_if #(D_W)      V_req       ();
axi_stream_if             S           ();
axi_stream_if #(D_W)      softmax_out ();
axi_stream_if #(D_W_ACC)  bias_C      ();
axi_stream_if #(D_W_ACC)  m_C         ();
axi_stream_if #(D_W_ACC)  e_C         ();
axi_stream_if             C           ();

assign bias_C.tdata = {D_W_ACC{1'b0}};
assign bias_C.tlast = 1'b1;
assign bias_C.tvalid = 1'b1;

attn_head_fsm #()
fsm_inst (
    .clk(clk),
    .rst(rst),
    .in_W_Q(in_W_Q),
    .in_W_K(in_W_K),
    .in_W_V(in_W_V),
    .in_bias_Q(in_bias_Q),
    .in_bias_K(in_bias_K),
    .in_bias_V(in_bias_V),
    .in_m_Q(in_m_Q),
    .in_m_K(in_m_K),
    .in_m_V(in_m_V),
    .in_e_Q(in_e_Q),
    .in_e_K(in_e_K),
    .in_e_V(in_e_V),
    .in_QKV_req(QKV_req),
    .out_W(W),
    .out_bias(bias),
    .out_m(m),
    .out_e(e),
    .out_Q_req(Q_req),
    .out_K_req(K_req),
    .out_V_req(V_req)
);

mm_pp #(
    .D_W(D_W),
    .D_W_ACC(D_W_ACC),
    .N1(N1),
    .N2(N2),
    .MATRIXSIZE_W(MATRIXSIZE_W),
    .KEEP_A(3 * KEEP_A),
    .MEM_DEPTH_A(MEM_DEPTH_A),
    .MEM_DEPTH_B(MEM_DEPTH_B),
    .MEM_DEPTH_D(MEM_DEPTH_D),
    .TRANSPOSE_B(0)
)
mm_QKV (
    .mm_clk(clk),
    .mm_fclk(fclk),
    .mm_rst_n(~rst),

    .in_A(in_I),
    .in_B(W),
    .out_D(QKV),

    .BLOCKS(mm_dimensions.BLOCKS),
    .BLOCK_WIDTH(mm_dimensions.BLOCK_WIDTH),
    .BLOCK_WIDTHdN2(mm_dimensions.BLOCK_WIDTHdN2),
    .BLOCK_SIZEdN2(mm_dimensions.BLOCK_SIZEdN2),
    .M1xBLOCK_WIDTHdN1xN2(mm_dimensions.M1xBLOCK_WIDTHdN1xN2),
    .M1xBLOCK_WIDTHdN1(mm_dimensions.M1xBLOCK_WIDTHdN1),
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
vec_mem_bias_QKV (
    .clk(clk),
    .rst(rst),

    .in_vec(bias),
    .out_vec(bias_QKV),

    .DIM1(mm_dimensions.M1),
    .DIM2(mm_dimensions.M3)
);

stream_vector_mem #(
    .D_W(D_W_ACC),
    .MATRIXSIZE_W(MATRIXSIZE_W),
    .MEM_DEPTH(REQ_MEM_DEPTH)
)
vec_mem_m_QKV (
    .clk(clk),
    .rst(rst),

    .in_vec(m),
    .out_vec(m_QKV),

    .DIM1(mm_dimensions.M1),
    .DIM2(mm_dimensions.M3)
);

stream_vector_mem #(
    .D_W(D_W),
    .MATRIXSIZE_W(MATRIXSIZE_W),
    .MEM_DEPTH(REQ_MEM_DEPTH)
)
vec_mem_e_QKV (
    .clk(clk),
    .rst(rst),

    .in_vec(e),
    .out_vec(e_QKV),

    .DIM1(mm_dimensions.M1),
    .DIM2(mm_dimensions.M3)
);

requant #(
    .D_W(D_W),
    .D_W_ACC(D_W_ACC),
    .OUT_BITS(D_W),
    .CLIP(1)
)
requant_QKV (
    .clk(clk),
    .rst(rst),

    .in_A(QKV),
    .in_bias(bias_QKV),
    .in_m(m_QKV),
    .in_e(e_QKV),

    .out_D(QKV_req)
);

mm #(
    .D_W(D_W),
    .D_W_ACC(D_W_ACC),
    .N1(sN1),
    .N2(sN2),
    .MATRIXSIZE_W(MATRIXSIZE_W),
    .MEM_DEPTH_A(MEM_DEPTH_Q),
    .MEM_DEPTH_B(MEM_DEPTH_K),
    .MEM_DEPTH_D(MEM_DEPTH_S),
    .TRANSPOSE_B(1)
)
mm_S (
    .mm_clk(clk),
    .mm_fclk(fclk),
    .mm_rst_n(~rst),

    .in_A(Q_req),
    .in_B(K_req),
    .out_D(S),

    .M2(64),
    .M3(32),
    .M1xM3dN1(32*32/sN1),
    .M1dN1(32/sN1),
    .M3dN2(32/sN2),
    .M1xM3dN1xN2(32*32/(sN1*sN2))
);

softmax_top #(
    .D_W(D_W),
    .D_W_ACC(D_W_ACC),
    .MATRIXSIZE_W(MATRIXSIZE_W),
    .LAYERS(LAYERS),
    .N(SOFTMAX_N)
)
softmax_top_unit (
    .clk(clk),
    .rst(rst),
    .layer(layer),

    .qin(S),
    .qout(softmax_out)
);

mm #(
    .D_W(D_W),
    .D_W_ACC(D_W_ACC),
    .N1(sN1),
    .N2(sN2),
    .MATRIXSIZE_W(MATRIXSIZE_W),
    .MEM_DEPTH_A(MEM_DEPTH_P),
    .MEM_DEPTH_B(MEM_DEPTH_V),
    .MEM_DEPTH_D(MEM_DEPTH_C),
    .TRANSPOSE_B(0)
)
mm_C (
    .mm_clk(clk),
    .mm_fclk(fclk),
    .mm_rst_n(~rst),

    .in_A(softmax_out),
    .in_B(V_req),
    .out_D(C),

    .M2(32),
    .M3(64),
    .M1xM3dN1(32*64/sN1),
    .M1dN1(32/sN1),
    .M3dN2(64/sN2),
    .M1xM3dN1xN2(32*64/(sN1*sN2))
);

stream_scalar_mem #(
    .D_W(D_W_ACC),
    .MATRIXSIZE_W(MATRIXSIZE_W)
)
sca_mem_m_C (
    .clk(clk),
    .rst(rst),

    .in_sca(in_m_C),
    .out_sca(m_C),

    .DIM1(mm_dimensions.M1),
    .DIM2(mm_dimensions.M3)
);

stream_scalar_mem #(
    .D_W(D_W),
    .MATRIXSIZE_W(MATRIXSIZE_W)
)
sca_mem_e_C (
    .clk(clk),
    .rst(rst),

    .in_sca(in_e_C),
    .out_sca(e_C),

    .DIM1(mm_dimensions.M1),
    .DIM2(mm_dimensions.M3)
);

requant #(
    .D_W(D_W),
    .D_W_ACC(D_W_ACC),
    .OUT_BITS(D_W),
    .CLIP(1)
)
requant_C (
    .clk(clk),
    .rst(rst),

    .in_A(C),
    .in_bias(bias_C),
    .in_m(m_C),
    .in_e(e_C),

    .out_D(attn_head_out)
);

endmodule
