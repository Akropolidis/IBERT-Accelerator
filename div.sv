`timescale 1ps / 1ps

module div
#(
    parameter integer D_W = 32
)
(
    input  logic           clk,
    input  logic           rst,
    input  logic           in_valid,
    input  logic           enable,
    input  logic [D_W-1:0] divisor,
    input  logic [D_W-1:0] dividend,
    output logic [D_W-1:0] quotient,
    output logic           out_valid
);

typedef enum logic {INIT, COMP} state_t;
state_t state;

logic [D_W-1:0] remainder, divisor_r, quotient_r;
logic [5:0] msb;

logic [$clog2(D_W)-1:0] divisor_log2;
logic [$clog2(D_W)-1:0] remainder_log2;
lopd lopd_divisor (.in_data(divisor_r), .out_data(divisor_log2));
lopd lopd_remainder (.in_data(remainder), .out_data(remainder_log2));

assign msb = remainder_log2 - divisor_log2;

always_ff @(posedge clk) begin: run_stmc
    if (rst) begin
        state <= INIT;
    end else begin
        if (enable) begin
            case (state)
                INIT: begin
                    if (in_valid) begin
                        state <= COMP;
                    end
                end
                COMP: begin
                    if (remainder < divisor_r) begin
                        state <= INIT;
                    end
                end
                default: state <= INIT;
            endcase
        end
    end
end

always_ff @(posedge clk) begin: compute
    if (rst) begin
        quotient <= 0;
        out_valid <= 0;
    end else begin
        if (enable) begin
            case (state)
                INIT: begin
                    out_valid <= 0;
                    if (in_valid) begin
                        remainder <= dividend;
                        divisor_r <= divisor;
                        quotient_r <= 0;
                    end
                end
                COMP: begin
                    if (remainder >= divisor_r) begin
                        //msb = remainder_log2 - divisor_log2;

                        if (remainder < (divisor_r << msb)) begin
                            remainder <= remainder - (divisor_r << (msb - 1));
                            quotient_r <=  quotient_r + (1 << (msb-1));
                        end else begin
                            remainder <= remainder - (divisor_r << msb);
                            quotient_r <= quotient_r + (1 << msb);
                        end
                    end else begin
                        quotient <= quotient_r;
                        out_valid <= 1;
                    end
                end
                default: begin
                    quotient <= 0;
                    out_valid <= 0;
                end
            endcase
        end
    end
end

endmodule
