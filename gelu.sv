`timescale 1ps / 1ps

module gelu
#(
    parameter integer D_W   = 32,
    parameter integer SHIFT = 14
)
(
    input  logic                  clk,
    input  logic                  rst,
    input  logic                  in_valid,
    input  logic                  enable,
    input  logic signed [D_W-1:0] qin,           // gelu input

    input  logic signed [D_W-1:0] qb,            // coefficient
    input  logic signed [D_W-1:0] qc,            // coefficient
    input  logic signed [D_W-1:0] q1,            // coefficient

    output logic                  out_valid,
    output logic signed [D_W-1:0] qout           // gelu output
);

logic signed [D_W-1:0] qin_r, qin_r1, qin_r2, qin_r3, qin_r4, qin_r5, qin_r6;

logic signed [D_W-1:0] qb_r, qb_r1, qb_r2;
logic signed [D_W-1:0] q1_r, q1_r1, q1_r2, q1_r3, q1_r4, q1_r5, q1_r6;

//Intermediate signals
logic signed [D_W-1:0] qsgn;
logic signed [D_W-1:0] qin_abs;
logic signed [D_W-1:0] qmin, qmin_r3;
logic signed [D_W-1:0] ql, ql_r4, ql_r5;
logic signed [D_W-1:0] qerf;

logic in_valid_r, in_valid1, in_valid2, in_valid3, in_valid4, in_valid5, in_valid6;

always_ff @(posedge clk) begin
    if (rst) begin
        qin_r <= 0; qin_r1 <= 0; qin_r2 <= 0; qin_r3 <= 0; qin_r4 <= 0; qin_r5 <= 0; qin_r6 <= 0;
        qb_r <= 0; qb_r1 <= 0; qb_r2 <= 0;
        q1_r <= 0; q1_r1 <= 0; q1_r2 <= 0; q1_r3 <= 0; q1_r4 <= 0; q1_r5 <= 0; q1_r6 <= 0;

        qsgn <= 0;
        qin_abs <= 0;
        qmin <= 0; qmin_r3 <= 0;
        ql <= 0; ql_r4 <= 0; ql_r5 <= 0;
        qerf <= 0;

        qout <= 0;
        in_valid_r <= 0; in_valid1 <= 0; in_valid2 <= 0; in_valid3 <= 0; in_valid4 <= 0; in_valid5 <= 0; in_valid6 <= 0;
        out_valid <= 0;
    end else begin
        if (enable) begin
            // input pipelining
            in_valid_r <= in_valid;
            qin_r <= qin;
            qb_r <= qb;
            q1_r <= q1;

            // stage 1
            in_valid1 <= in_valid_r;
            // qsgn <= (qin_r[D_W-1]) ? -1 : 1; //Checking MSB, if 1 (neg), 0 (pos)
            qb_r1 <= qb_r;
            q1_r1 <= q1_r;
            qin_r1 <= qin_r;
            qin_abs <= (qin_r[D_W-1]) ? -qin_r : qin_r; //Computing abs(qin)

            // stage 2
            in_valid2 <= in_valid1;
            qmin <= (qin_abs < -qb_r1) ? qin_abs : -qb_r1;
            qb_r2 <= qb_r1;
            q1_r2 <= q1_r1;
            qin_r2 <= qin_r1;

            // stage 3
            in_valid3 <= in_valid2;
            ql <= (qmin + (qb_r2 <<< 1));
            q1_r3 <= q1_r2;
            qin_r3 <= qin_r2;
            qmin_r3 <= qmin;

            // stage 4
            in_valid4 <= in_valid3;
            ql_r4 <= ql * qmin_r3;
            q1_r4 <= q1_r3;
            qin_r4 <= qin_r3;

            // stage 5
            in_valid5 <= in_valid4;
            ql_r5 <= ql_r4 + qc;
            q1_r5 <= q1_r4;
            qin_r5 <= qin_r4;
            qsgn <= (qin_r4[D_W-1]) ? -1 : 1; //Checking MSB, if 1 (neg), 0 (pos)
            
            // stage 6
            in_valid6 <= in_valid5;
            qerf <= (qsgn * ql_r5) >>> SHIFT;
            q1_r6 <= q1_r5;
            qin_r6 <= qin_r5;

            // stage 7
            out_valid <= in_valid6;
            qout <= qin_r6 * (qerf + q1_r6);
        end
    end
end

endmodule
