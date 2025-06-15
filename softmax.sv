`timescale 1ps / 1ps

module softmax
#(
    parameter integer D_W      = 8,
    parameter integer D_W_ACC  = 32,
    parameter integer N        = 32,
    parameter integer FP_BITS  = 30,
    parameter integer MAX_BITS = 30,
    parameter integer OUT_BITS = 6
)
(
    input  logic                      clk,
    input  logic                      rst,
    input  logic                      enable,
    input  logic                      in_valid,
    input  logic signed [D_W_ACC-1:0] qin,           // softmax input
    input  logic signed [D_W_ACC-1:0] qb,            // exp coefficient
    input  logic signed [D_W_ACC-1:0] qc,            // exp coefficient
    input  logic signed [D_W_ACC-1:0] qln2,          // exp coefficient
    input  logic signed [D_W_ACC-1:0] qln2_inv,      // exp coefficient
    input  logic        [D_W_ACC-1:0] Sreq,
    output logic                      out_valid,
    output logic signed [D_W-1:0]     qout           // softmax output
);

localparam integer LATENCY_EXP = 7;
localparam integer LATENCY_DIV = 8;
logic signed [D_W_ACC-1:0] qout1, out_counter;
//Helper signals for instantiating modules
logic signed [D_W_ACC-1:0] qout_div;
logic qhat_valid_r1, qhat_valid_r2, qhat_valid, out_valid_exp, qexp_64_valid, qreq_valid, acc_done, out_valid_div, q_factor_valid;

//Intermediate signals from python formula
logic signed [D_W_ACC-1:0] divident;
logic signed [D_W_ACC-1:0] shift;
logic signed [D_W_ACC-1:0] qmax, fifo_max_out_r, fifo_max_out;
logic signed [16-1:0] fifo_acc_out_r, fifo_acc_out, fifo_div_out_r, fifo_div_out;
logic signed [D_W_ACC-1:0] qhat;
logic signed [D_W_ACC-1:0] qexp_32;
logic signed [2*D_W_ACC-1:0] qexp_64;
logic signed [D_W_ACC-1:0] qreq_r, qreq;
logic signed [16-1:0] qreq_int16;
logic signed [D_W_ACC-1:0] qsum;
logic signed [D_W_ACC-1:0] factor;
logic signed [D_W_ACC-1:0] accum;

//Signals for max FIFO module buffering
logic fifo_max_write;
logic fifo_max_read;
logic fifo_max_full, fifo_max_empty;
logic signed [D_W_ACC-1:0] qmax_interim;

logic signed [D_W_ACC-1:0] max_counter;
logic [D_W_ACC-1:0] fifo_max_occupancy; //Tracks how many elements are in the max FIFO
logic init_max;

//Signals for acc FIFO module buffering
logic fifo_acc_write;
logic fifo_acc_read;
logic fifo_acc_full, fifo_acc_empty;
logic signed [D_W_ACC-1:0] qsum_interim;

logic signed [D_W_ACC-1:0] acc_counter;
logic [D_W_ACC-1:0] fifo_acc_occupancy; //Tracks how many elements are in the acc FIFO
logic init_acc, qsum_indicator;

//Signals for div FIFO module buffering
logic fifo_div_write;
logic fifo_div_read;
logic fifo_div_full, fifo_div_empty;
logic signed [D_W_ACC-1:0] qdiv_interim;

logic signed [D_W_ACC-1:0] div_counter;
logic [D_W_ACC-1:0] fifo_div_occupancy; //Tracks how many elements are in the div FIFO
logic init_div;
max #(
    .D_W(D_W_ACC)
) max_inst (
    .clk(clk),
    .rst(rst),
    .enable(enable),
    .initialize(init_max),
    .in_data(qin),
    .result(qmax_interim));
// acc #(
//     .D_W(D_W_ACC),
//     .D_W_ACC(D_W_ACC)
// ) acc_inst (
//     .clk(clk),
//     .rst(rst),
//     .enable(enable),
//     .initialize(init_acc),
//     .in_data(qreq_int16),
//     .result(qsum_interim));
exp #(
    .D_W(D_W_ACC),
    .FP_BITS(FP_BITS)
) exp_inst (
    .clk(clk),
    .rst(rst),
    .in_valid(qhat_valid),
    .enable(enable),
    .qin(qhat),
    .qb(qb),
    .qc(qc),
    .qln2(qln2),
    .qln2_inv(qln2_inv),
    .out_valid(out_valid_exp),
    .qout(qexp_32));
div #(
    .D_W(D_W_ACC)
) div_inst (
    .clk(clk),
    .rst(rst),
    .in_valid(acc_done),
    .enable(enable),
    .divisor(qsum),
    .dividend(divident),
    .quotient(qdiv_interim),
    .out_valid(out_valid_div));
fifo #(
    .D_W(D_W_ACC),
    .DEPTH(N)
) fifo_max_inst (
    .clk(clk),
    .rst(rst),
    .write(fifo_max_write),
    .read(fifo_max_read),
    .data_in(qin),
    .data_out(fifo_max_out_r),
    .full(fifo_max_full),
    .empty(fifo_max_empty));
fifo #(
    .D_W(16),
    .DEPTH(N)
) fifo_acc_inst (
    .clk(clk),
    .rst(rst),
    .write(fifo_acc_write),
    .read(fifo_acc_read),
    .data_in(qreq_int16),
    .data_out(fifo_acc_out_r),
    .full(fifo_acc_full),
    .empty(fifo_acc_empty)
);
fifo #(
    .D_W(16),
    .DEPTH(N)
) fifo_div_inst (
    .clk(clk),
    .rst(rst),
    .write(fifo_div_write),
    .read(fifo_div_read),
    .data_in(fifo_acc_out),
    .data_out(fifo_div_out_r),
    .full(fifo_div_full),
    .empty(fifo_div_empty)
);
typedef enum logic [1:0] {
    RUN,
    FLUSH,
    WAIT
} state_t;

state_t current_state, next_state;
logic [D_W_ACC-1:0] flush_counter;

// Sequential state update
always_ff @(posedge clk) begin
    if (rst)
        current_state <= RUN;
    else
        current_state <= next_state;
end

//Next-state logic
always_comb begin
    next_state = current_state;
    case (current_state)
        RUN: begin
            //If enable goes low, start flush
            if (out_valid_counter == (N*N - 1))
                next_state = FLUSH;
        end
        FLUSH: begin
            if (flush_counter == N)
                next_state = WAIT;
        end
        WAIT: begin
            if (in_valid)
                next_state = RUN;
        end
        default: next_state = RUN;
    endcase
end

always_ff @(posedge clk) begin
    if (rst)
        flush_counter <= 0;
    else if (current_state == FLUSH)
        flush_counter <= flush_counter + 1;
    else
        flush_counter <= 0;
end

logic batch_active;
always_ff @(posedge clk) begin
if (rst)
    batch_active <= 0;
else if (in_valid)
    batch_active <= 1;
else if (!in_valid && fifo_max_empty && fifo_acc_empty && fifo_div_empty)
    batch_active <= 0;
end


assign fifo_max_write = (current_state == RUN || (next_state == RUN)) && (in_valid || batch_active) && (fifo_max_occupancy < N);
assign fifo_max_read = ((current_state == RUN) || (current_state == FLUSH)) &&
                         (fifo_max_occupancy > 0) &&
                         ((current_state == FLUSH) ? 1'b1 : (fifo_max_full || init_max));
assign fifo_acc_write = (current_state == RUN) && (qreq_valid) && (fifo_acc_occupancy < N);
assign fifo_acc_read = ((current_state == RUN) || (current_state == FLUSH)) &&
                         (fifo_acc_occupancy > 0) &&
                         ((current_state == FLUSH) ? 1'b1 : (fifo_acc_full || init_acc));

logic acc_done_latched;
always_ff @(posedge clk) begin
    if (rst) begin
        acc_done_latched <= 0;
    end else if (enable) begin
        if (acc_done) begin
            acc_done_latched <= 1;
        end else if (current_state == FLUSH) begin
            acc_done_latched <= 0;
        end
    end
end

assign fifo_div_write = (current_state == RUN) && acc_done_latched && (fifo_div_occupancy < N);

assign fifo_div_read = ((current_state == RUN) || (current_state == FLUSH)) &&
                         (fifo_div_occupancy > 0) &&
                         ((current_state == FLUSH) ? 1'b1 : (fifo_div_full));

assign init_max = (max_counter == 0) && (in_valid || batch_active);
assign init_acc = (acc_counter == 0) && (qreq_valid);
assign init_div = (div_counter == 0) && (acc_done_latched);


always_ff @(posedge clk) begin
    if (rst) begin
        fifo_max_occupancy <= 0;
    end else begin
        case ({fifo_max_write, fifo_max_read})
            2'b10: fifo_max_occupancy <= fifo_max_occupancy + 1;
            2'b01: fifo_max_occupancy <= fifo_max_occupancy - 1;
            default: fifo_max_occupancy <= fifo_max_occupancy;
        endcase
    end
end
always_ff @(posedge clk) begin
    if (rst) begin
        fifo_acc_occupancy <= 0;
    end else begin
        case ({fifo_acc_write, fifo_acc_read})
            2'b10: fifo_acc_occupancy <= fifo_acc_occupancy + 1;
            2'b01: fifo_acc_occupancy <= fifo_acc_occupancy - 1;
            default: fifo_acc_occupancy <= fifo_acc_occupancy;
        endcase
    end
end
always_ff @(posedge clk) begin
    if (rst) begin
        fifo_div_occupancy <= 0;
    end else begin
        case ({fifo_div_write, fifo_div_read})
            2'b10: fifo_div_occupancy <= fifo_div_occupancy + 1;
            2'b01: fifo_div_occupancy <= fifo_div_occupancy - 1;
            default: fifo_div_occupancy <= fifo_div_occupancy;
        endcase
    end
end

///////////////////////////COUNTERS//////////////////////////
always_ff @(posedge clk) begin
    if (rst || (current_state == FLUSH)) begin
        max_counter <= 0;
    end else if ((enable) && fifo_max_write) begin
        if (max_counter == N-1)
            max_counter <= 0;
        else
            max_counter <= max_counter + 1;
    end
end
always_ff @(posedge clk) begin
    if (rst || (current_state == FLUSH)) begin
        acc_counter <= 0;
    end else if ((enable) && fifo_acc_write) begin
        if (acc_counter == N-1)
            acc_counter <= 0;
        else
            acc_counter <= acc_counter + 1;
    end
end
always_ff @(posedge clk) begin
    if (rst || (current_state == FLUSH)) begin
        div_counter <= 0;
    end else if ((enable) && fifo_div_write) begin
        if (div_counter == N-1)
            div_counter <= 0;
        else
            div_counter <= div_counter + 1;
    end
end
/////////////////////////////////////////////////////////////
always_ff @(posedge clk) begin
    if (rst) begin
        qmax <= 0;
    end else if (enable) begin
        if (init_max)
            qmax <= qmax_interim;
        else
            qmax <= qmax;
    end
end
assign qsum = (qsum_indicator) ? qsum_interim : 0;
always_ff @(posedge clk) begin
    if (rst) begin
        factor <= 0;
    end else if (enable) begin
        if (init_div)
            factor <= qdiv_interim;
        else
            factor <= factor;
    end
end


///////////ACCUMULATOR IMPLEMENTATION//////////////
always_ff @(posedge clk) begin
    if (rst) begin
        qsum_interim <= 0;
        qsum_indicator <= 0;
    end else begin
        if (enable) begin
            qsum_indicator <= init_acc;
            qsum_interim <= accum;
        end
    end
end
always_comb begin
    if(qsum_indicator) begin
        accum = qreq_int16;
    end else begin
        accum = qsum_interim + qreq_int16;
    end
end
///////////////////////////////////////////////////

assign qreq_int16 = qreq[16-1:0];
assign divident = 1 << MAX_BITS;
assign shift = MAX_BITS - OUT_BITS;

////////////////////ACTUAL SOFTMAX IMPLEMENTATION////////////////////
always_ff @(posedge clk) begin
    if (rst || (current_state == FLUSH)) begin
        out_counter <= 0;
        qhat_valid_r1 <= 0;
        qhat_valid_r2 <= 0;
        qhat_valid <= 0;
        acc_done <= 0;

        fifo_max_out <= 0;
        fifo_acc_out <= 0;
        qhat <= 0;
        qexp_64 <= 0;
        qreq_r <= 0;
        qreq <= 0;
        qreq_valid <= 0;
    end else begin
        if (enable) begin
            //stage 3
            if (fifo_max_read) begin
                fifo_max_out <= fifo_max_out_r;
                qhat <= fifo_max_out - qmax;
                qhat_valid_r1 <= 1;
                qhat_valid_r2 <= qhat_valid_r1;
                qhat_valid <= qhat_valid_r2;
            end else begin
                qhat_valid <= 0;
            end

            // stage 4
            if (out_valid_exp) begin
                qexp_64 <= qexp_32 * Sreq;
                qexp_64_valid <= out_valid_exp;
            end else begin
                qexp_64 <= 0;
                qexp_64_valid <= 0;
            end

            // stage 5
            if (qexp_64_valid) begin
                qreq_r <= (qexp_64 + (1 << (FP_BITS - 1))) >> FP_BITS; //left shift by (FP_BITS - 1) before diving by FP_BITS does rounding
                if ((qexp_64 & ((1 << FP_BITS) - 1)) == (1 << (FP_BITS - 1))) begin
                    qreq <= (qreq_r & 1) ? qreq_r - 1 : qreq_r;
                end else begin
                    qreq <= qreq_r; // Regular rounding result
                end
                qreq_valid <= 1;
            end else begin
                qreq <= 0;
                qreq_valid <= 0;
            end

            // stage 6
            if (fifo_acc_read) begin
                fifo_acc_out <= fifo_acc_out_r;
                acc_done <= init_acc;
            end

            //stage 7: Update FIFO-div output and valid flag
            if ((current_state == RUN) && fifo_div_read) begin
                fifo_div_out <= fifo_div_out_r;
                q_factor_valid <= 1;
            end else begin
                fifo_div_out <= 0;
                q_factor_valid <= 0;
            end
        end

        if (out_valid && (current_state==RUN)) begin
            out_counter <= out_counter + 1;
        end else begin
            out_counter <= 0;
        end
    end 
end
// Use a counter that can count up to N*N cycles
logic [D_W_ACC-1:0] out_valid_counter;
logic consumed_valid;
always_ff @(posedge clk) begin
    if (rst) begin
        out_valid_counter <= 0;
        out_valid <= 0;
        consumed_valid <= 0;
    end else begin
        if (current_state == RUN) begin
            if (q_factor_valid) begin
                if (!out_valid && !consumed_valid) begin
                    out_valid <= 1;
                    out_valid_counter <= 0;
                end else begin
                    out_valid_counter <= out_valid_counter + 1;
                    if (out_valid_counter == (N*N - 1)) begin
                        out_valid <= 0;
                        out_valid_counter <= 0;
                        consumed_valid <= 1;
                    end
                end
            end else begin
                out_valid <= 0;
                out_valid_counter <= 0;
                consumed_valid <= 0;
            end
        end else begin
            out_valid_counter <= 0;
            out_valid <= 0;
        end
    end
end

assign qout1 = (q_factor_valid) ? fifo_div_out * factor : 'x;
assign qout = qout1 >> shift;

endmodule
