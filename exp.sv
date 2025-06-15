`timescale 1ps / 1ps

module exp
#(
    parameter integer D_W     = 32,
    parameter integer FP_BITS = 30
)
(
    input  logic                  clk,
    input  logic                  rst,
    input  logic                  in_valid,
    input  logic                  enable,
    input  logic signed [D_W-1:0] qin,           // exp input

    input  logic signed [D_W-1:0] qb,            // coefficient
    input  logic signed [D_W-1:0] qc,            // coefficient
    input  logic signed [D_W-1:0] qln2,          // coefficient
    input  logic signed [D_W-1:0] qln2_inv,      // coefficient

    output logic                  out_valid,
    output logic signed [D_W-1:0] qout           // exp output
);

logic signed [D_W-1:0] qin_r, qin_r1, qin_r2;

logic signed [D_W-1:0] qb_r, qb_r1, qb_r2, qb_r3;
logic signed [D_W-1:0] qc_r, qc_r1, qc_r2, qc_r3, qc_r4, qc_r5;
logic signed [D_W-1:0] qln2_r, qln2_r1;
logic signed [D_W-1:0] qln2_inv_r;

//Intermediate signals
logic signed [2*D_W-1:0] qout1_r, qout2_r, qout3_r, qout4_r, qout5_r, qout6_r;
logic signed [2*D_W-1:0] qout1_r2, qout1_r3, qout1_r4, qout1_r5, qout1_r6;
logic signed [2*D_W-1:0] qout3_r4;

logic in_valid_r, in_valid1, in_valid2, in_valid3, in_valid4, in_valid5, in_valid6;

always_ff @(posedge clk) begin
    if (rst) begin
        qin_r <= 0; qin_r1 <= 0; qin_r2 <= 0;
        qb_r <= 0; qb_r1 <= 0; qb_r2 <= 0; qb_r3 <= 0;
        qc_r <= 0; qc_r1 <= 0; qc_r2 <= 0; qc_r3 <= 0; qc_r4 <= 0; qc_r5 <= 0;
        qln2_r <= 0; qln2_r1 <= 0;
        qln2_inv_r <= 0;

        qout1_r <= 0; qout2_r <= 0; qout3_r <= 0; qout4_r <= 0; qout5_r <= 0; qout6_r <= 0;
        qout1_r2 <= 0; qout1_r3 <= 0; qout1_r4 <= 0; qout1_r5 <= 0; qout1_r6 <= 0;
        qout3_r4 <= 0;

        qout <= 0;
        in_valid_r <= 0; in_valid1 <= 0; in_valid2 <= 0; in_valid3 <= 0; in_valid4 <= 0; in_valid5 <= 0; in_valid6 <= 0;
        out_valid <= 0;
    end else begin
        if (enable) begin
            // input pipelining
            in_valid_r <= in_valid;
            qin_r <= qin;
            qb_r <= qb;
            qc_r <= qc;
            qln2_r <= qln2;
            qln2_inv_r <= qln2_inv;

            // stage 1
            in_valid1 <= in_valid_r;
            qout1_r <= qin_r * qln2_inv_r;
            qin_r1 <= qin_r;
            qb_r1 <= qb_r;
            qc_r1 <= qc_r;
            qln2_r1 <= qln2_r;

            // stage 2
            in_valid2 <= in_valid1;
            qout2_r <= qln2_r1 * (qout1_r >> FP_BITS);
            qin_r2 <= qin_r1;
            qb_r2 <= qb_r1;
            qc_r2 <= qc_r1;
            qout1_r2 <= qout1_r;

            // stage 3
            in_valid3 <= in_valid2;
            qout3_r <= qin_r2 - qout2_r;
            qb_r3 <= qb_r2;
            qc_r3 <= qc_r2;
            qout1_r3 <= qout1_r2;

            // stage 4
            in_valid4 <= in_valid3;
            qout4_r <= qout3_r + qb_r3;
            qout3_r4 <= qout3_r; //shift since need to use later
            qc_r4 <= qc_r3;
            qout1_r4 <= qout1_r3;


            // stage 5
            in_valid5 <= in_valid4;
            qout5_r <= qout3_r4 * qout4_r;
            qc_r5 <= qc_r4;
            qout1_r5 <= qout1_r4;

            // stage 6
            in_valid6 <= in_valid5;
            qout6_r <= qout5_r + qc_r5;
            qout1_r6 <= qout1_r5;

            //stage 7
            out_valid <= in_valid6;
            qout <= qout6_r >> (qout1_r6 >> FP_BITS);
        end
    end
end

endmodule
