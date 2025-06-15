`timescale 1ps / 1ps

module mm
#(
    parameter integer D_W          = 8,
    parameter integer D_W_ACC      = 32,
    parameter integer N1           = 4,
    parameter integer N2           = 4,
    parameter integer MATRIXSIZE_W = 24,
    parameter integer KEEP_A       = 1,
    parameter integer MEM_DEPTH_A  = 6144,
    parameter integer MEM_DEPTH_B  = 12288,
    parameter integer MEM_DEPTH_D  = 512,
    parameter integer TRANSPOSE_B  = 0
)
(
    input  wire                    mm_clk,
    input  wire                    mm_fclk,
    input  wire                    mm_rst_n,

    axi_stream_if.axi_in           in_A,
    axi_stream_if.axi_in           in_B,
    axi_stream_if.axi_out          out_D,

    input  wire [MATRIXSIZE_W-1:0] M2,
    input  wire [MATRIXSIZE_W-1:0] M3,
    input  wire [MATRIXSIZE_W-1:0] M1xM3dN1,
    input  wire [MATRIXSIZE_W-1:0] M1dN1,
    input  wire [MATRIXSIZE_W-1:0] M3dN2,
    input  wire [MATRIXSIZE_W-1:0] M1xM3dN1xN2
);

localparam integer ADDR_W_A = $clog2(MEM_DEPTH_A);
localparam integer ADDR_W_B = $clog2(MEM_DEPTH_B);
localparam integer ADDR_W_D = $clog2(MEM_DEPTH_D);

wire clk;
wire fclk;
wire rst;

assign clk  = mm_clk;
assign fclk = mm_fclk;
assign rst  = ~mm_rst_n;

wire        [N2-1:0]       init   [N1-1:0];
wire signed [D_W-1:0]      A_bram [N1-1:0];
wire signed [D_W-1:0]      B_bram [N2-1:0];
wire signed [D_W_ACC-1:0]  data_D [N1-1:0];
wire        [N1-1:0]       valid_D;

wire        [ADDR_W_A-1:0] rd_addr_A;
wire        [ADDR_W_B-1:0] rd_addr_B;

wire                       done_multiply;
wire                       start_multiply;

s2mm #(
    .D_W          (D_W),
    .N1           (N1),
    .N2           (N2),
    .MATRIXSIZE_W (MATRIXSIZE_W),
    .KEEP_A       (KEEP_A),
    .MEM_DEPTH_A  (MEM_DEPTH_A),
    .MEM_DEPTH_B  (MEM_DEPTH_B),
    .ADDR_W_A     (ADDR_W_A),
    .ADDR_W_B     (ADDR_W_B),
    .TRANSPOSE_B  (TRANSPOSE_B)
)
s2mm_inst (
    .clk                  (clk),
    .fclk                 (fclk),
    .rst                  (rst),
    .s_axis_s2mm_tdata_A  ({{(D_W_ACC-D_W){in_A.tdata[D_W-1]}}, in_A.tdata}),
    .s_axis_s2mm_tlast_A  (in_A.tlast),
    .s_axis_s2mm_tready_A (in_A.tready),
    .s_axis_s2mm_tvalid_A (in_A.tvalid),
    .s_axis_s2mm_tdata_B  ({{(D_W_ACC-D_W){in_B.tdata[D_W-1]}}, in_B.tdata}),
    .s_axis_s2mm_tlast_B  (in_B.tlast),
    .s_axis_s2mm_tready_B (in_B.tready),
    .s_axis_s2mm_tvalid_B (in_B.tvalid),
    .rd_addr_A            (rd_addr_A),
    .rd_addr_B            (rd_addr_B),
    .A_bram               (A_bram),
    .B_bram               (B_bram),
    .M2                   (M2),
    .M1dN1                (M1dN1),
    .M3dN2                (M3dN2),
    .done_multiply        (done_multiply),
    .start_multiply       (start_multiply)
);

mm2s #(
    .D_W_ACC      (D_W_ACC),
    .N1           (N1),
    .N2           (N2),
    .ADDR_W_D     (ADDR_W_D),
    .MATRIXSIZE_W (MATRIXSIZE_W),
    .MEM_DEPTH_D  (MEM_DEPTH_D)
)
mm2s_inst (
    .clk                (clk),
    .fclk               (fclk),
    .rst                (rst),
    .m_axis_mm2s_tdata  (out_D.tdata),
    .m_axis_mm2s_tlast  (out_D.tlast),
    .m_axis_mm2s_tready (out_D.tready),
    .m_axis_mm2s_tvalid (out_D.tvalid),
    .data_D             (data_D),
    .valid_D            (valid_D),
    .M3                 (M3),
    .M1dN1              (M1dN1),
    .M1xM3dN1           (M1xM3dN1),
    .done_multiply      (done_multiply)
);

control #(
    .N1           (N1),
    .N2           (N2),
    .MATRIXSIZE_W (MATRIXSIZE_W),
    .ADDR_W_A     (ADDR_W_A),
    .ADDR_W_B     (ADDR_W_B)
)
control_inst (
    .clk         (fclk),
    .rst         (~start_multiply),
    .M2          (M2),
    .M1dN1       (M1dN1),
    .M3dN2       (M3dN2),
    .M1xM3dN1xN2 (M1xM3dN1xN2),
    .rd_addr_A   (rd_addr_A),
    .rd_addr_B   (rd_addr_B),
    .init        (init)
);

systolic #(
    .D_W     (D_W),
    .D_W_ACC (D_W_ACC),
    .N1      (N1),
    .N2      (N2)
)
systolic_inst (
    .clk     (fclk),
    .rst     (~start_multiply),
    .init    (init),
    .A       (A_bram),
    .B       (B_bram),
    .D       (data_D),
    .valid_D (valid_D)
);

endmodule
