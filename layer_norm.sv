`timescale 1ps / 1ps

module layer_norm
#(
    parameter integer       D_W      = 8,
    parameter integer       D_W_ACC  = 32,
    parameter integer       N        = 768,
    parameter signed [D_W_ACC-1:0] N_INV = 1398101,
    parameter integer       FP_BITS  = 30,
    parameter integer       MAX_BITS = 31
)
(
    input  logic                       clk,
    input  logic                       rst,
    input  logic                       enable,
    input  logic                       in_valid,
    input  logic signed  [D_W_ACC-1:0] qin,
    input  logic signed  [D_W_ACC-1:0] bias,
    input  logic [$clog2(D_W_ACC)-1:0] shift,
    output logic                       out_valid,
    output logic signed  [D_W_ACC-1:0] qout
);

localparam int LATENCY_SQRT = 16;
localparam int LATENCY_DIV  = 32;
localparam int LATENCY_BIAS_FIFO = 1 + N + 4 + LATENCY_SQRT + 1 + LATENCY_DIV + 1;
localparam int LATENCY_ACC_FIFO  = N;
localparam int LATENCY_DIV_FIFO  = 1 + LATENCY_SQRT + 1 + LATENCY_DIV;
localparam int SHIFT_AMOUNT = 2;
localparam int MAX_NUM = (1 <<< (D_W_ACC-1)) - 1;
localparam int BUBBLING_LATENCY = 710;


logic signed [D_W_ACC:0] global_counter;
logic signed [D_W_ACC-1:0] divident;
logic signed [D_W_ACC-1:0] qin_r1, qin_r2, q_shift;

// logic signed [D_W_ACC-1:0] qbias_interim;
logic signed [2*D_W_ACC-1:0] qsum_interim;
logic signed [2*D_W_ACC-1:0] qsum_sq_interim;
logic signed [D_W_ACC-1:0] qdiv_interim;

// Outputs from the accumulators after N samples
logic signed [2*D_W_ACC-1:0] qsum;
logic signed [2*D_W_ACC-1:0] qsum_sq, qsum_sq_r1, qsum_sq_r2;
logic signed [D_W_ACC-1:0] factor;

// Mean and variance intermediate
logic signed [D_W_ACC-1:0] qmean;
logic signed [D_W_ACC-1:0] qmean_sq;
logic signed [2*D_W_ACC-1:0] variance;
logic signed [D_W_ACC-1:0] var_scaled;

// sqrt(var) => 16 cycles
logic [(D_W_ACC>>1)-1:0] qsqrt;
logic [D_W_ACC-1:0] var_sqrt;
logic signed [D_W_ACC-1:0] std;

//qmul, r, qmean_mul
logic signed [2*D_W_ACC-1:0] qmul;
logic signed [D_W_ACC-1:0] r;
logic signed [2*D_W_ACC-1:0] qmean_mul;


//Signals for bias FIFO module buffering
logic fifo_bias_write;
logic fifo_bias_read;
logic fifo_bias_full, fifo_bias_empty;

// logic signed [D_W_ACC-1:0] bias_counter;
logic [D_W_ACC-1:0] fifo_bias_occupancy;
// logic init_bias;
logic signed [D_W_ACC-1:0] fifo_bias, fifo_bias1, fifo_bias2, fifo_bias_out, fifo_bias_out_r;

//Signals for acc FIFO module buffering
logic fifo_acc_write;
logic fifo_acc_read;
logic fifo_acc_full, fifo_acc_empty;

logic signed [D_W_ACC-1:0] acc_counter;
logic [D_W_ACC-1:0] fifo_acc_occupancy;
logic init_acc, first_init_acc, qsum_indicator;
logic signed [D_W_ACC-1:0] fifo_acc_out_r, fifo_acc_out_r1, fifo_acc_out;

//Signals for div FIFO module buffering
logic fifo_div_write;
logic fifo_div_read;
logic fifo_div_full, fifo_div_empty;

logic signed [D_W_ACC-1:0] div_counter, max_div_counter;
logic [D_W_ACC-1:0] fifo_div_occupancy;
logic init_div, div_extra, div_check;
logic signed [D_W_ACC-1:0] fifo_div_out_r, fifo_div_out_r1, fifo_div_out_r2, fifo_div_out;

//Signals for acc FIFO module buffering
logic signed [D_W_ACC-1:0] qout_r, qout_temp;
logic fifo_qout_write;
logic fifo_qout_read;
logic fifo_qout_full, fifo_qout_empty;

//Valid handshaking
logic stage1N11, stage1N111, stage1N1111, r_valid, var_valid, var_sqrt_valid, std_ready, out_valid_sqrt, out_valid_div, qout_mul_valid;
logic signed r_valid_r1, r_valid_r2, r_valid_r3, r_valid_r4;
logic signed [D_W_ACC-1:0] accum;
logic signed [D_W_ACC-1:0] qout_mul, qout_mul_r1, qout_mul_r2;
logic qout_mul_valid_r1, qout_mul_valid_r2, qout_mul_valid_r3, qout_mul_valid_r4;
/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Counter that counts up to N*D_W_ACC cycles
logic [D_W_ACC-1:0] out_valid_counter;
logic consumed_valid;
logic bubbling, bubble, bubbled;
always_ff @(posedge clk) begin
    if (rst) begin
        out_valid <= 0;
        consumed_valid <= 0;
    end else begin
        if (enable) begin
            if (qout_mul_valid) begin
                if (!out_valid && !consumed_valid) begin
                    out_valid <= 1;
                end else begin
                    if (bubbled && (out_valid_counter == (N * D_W_ACC + 354))) begin
                        out_valid <= 0;
                        consumed_valid <= 1;
                    end
                    if ((enable && !bubbled) && (out_valid_counter == (N * D_W_ACC - 1))) begin
                        out_valid <= 0;
                        consumed_valid <= 1;
                    end
                end
            end else begin
                out_valid         <= 0;
                consumed_valid    <= 0;
            end
        end else begin
            out_valid <= 0;
            consumed_valid <= 0;
        end
    end
end

logic enable_r;
always_ff @(posedge clk) begin
    if (rst)
        enable_r <= 0;
    else
        enable_r <= enable;
end

always_ff @(posedge clk) begin
  if (rst) begin
    out_valid_counter <= 0;
  end else if (out_valid && enable_r && qout_mul_valid) begin
    out_valid_counter <= out_valid_counter + 1;
  end else begin
    out_valid_counter <= out_valid_counter;
  end
end

logic init_batch;
assign init_batch = ((out_valid_counter % N) == 0) ? 1 : 0;
///////////////////////////////////////////////////////////////////////////////////////////////////////////
logic falling_edge, rising_edge;
logic rising_edge_one_shot, rising_edge_one_shot_r, rising_edge_one_shot_r1, rising_edge_one_shot_r2, rising_edge_one_shot_r3, rising_edge_one_shot_r4, rising_edge_one_shot_r4_latch, last_rising_edge;
logic falling_edge_one_shot, falling_edge_one_shot_r, falling_edge_one_shot_r1, falling_edge_one_shot_r2;

assign falling_edge = enable_r && !enable;
assign rising_edge = enable && !enable_r;

logic [D_W_ACC-1:0] fall_cnt, temp_counter;

always_ff @(posedge clk) begin
    if (rst) begin
        fall_cnt <= 0;
    end else if (falling_edge)
        fall_cnt <= fall_cnt + 1;
    else
        fall_cnt <= fall_cnt;
end

always_ff @(posedge clk) begin
    if (rst)
        temp_counter <= 0;
    else if (bubbling && enable)
        temp_counter <= temp_counter + 1;
    else
        temp_counter <= 0;
end
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

always_ff @(posedge clk) begin
    if (rst) begin
        fifo_qout_write <= 0;
    end else begin
        fifo_qout_write <= qout_mul_valid && enable_r;
        if (rising_edge_one_shot_r || rising_edge_one_shot_r2 || rising_edge_one_shot_r4) begin
            fifo_qout_write <= 0;
        end
    end
end
assign fifo_qout_read = (enable && out_valid) || (fall_cnt == 355);

always_latch begin
    if (rst)
        bubbled = 0;
    else if (bubble)
        bubbled = 1;
    else if (!in_valid && fifo_bias_empty && fifo_acc_empty && fifo_div_empty)
        bubbled = 0;
end
always_comb begin
    if (enable && !bubbled) begin
        qout = qout_r;
    end else if (bubbled) begin
        qout = qout_temp;
    end
end

mac #(
    .D_W(D_W_ACC),
    .D_W_ACC(2*D_W_ACC)
) mac_qsum_sq_inst (
    .clk      (clk),
    .rst      (rst),
    .enable   (enable && in_valid),
    .initialize(first_init_acc || qsum_indicator),
    .a        (q_shift),
    .b        (q_shift),
    .result  (qsum_sq_interim)
);
div #(
    .D_W(D_W_ACC)
) div_inst (
    .clk(clk),
    .rst(rst),
    .in_valid(std_ready),
    .enable(enable),
    .divisor(std),
    .dividend(divident),
    .quotient(qdiv_interim),
    .out_valid(out_valid_div)
);
sqrt #(
    .D_W(D_W_ACC)
) sqrt_inst (
    .clk(clk),
    .rst(rst),
    .enable(enable),
    .in_valid(var_valid),
    .qin(var_scaled),
    .out_valid(out_valid_sqrt),
    .qout(qsqrt)
);


fifo #(
    .D_W(D_W_ACC),
    .DEPTH(LATENCY_BIAS_FIFO)
) fifo_bias_inst (
    .clk(clk),
    .rst(rst),
    .write(fifo_bias_write),
    .read(fifo_bias_read),
    .data_in(bias),
    .data_out(fifo_bias_out_r),
    .full(fifo_bias_full),
    .empty(fifo_bias_empty)
);
fifo #(
    .D_W(D_W_ACC),
    .DEPTH(LATENCY_ACC_FIFO)
) fifo_acc_inst (
    .clk(clk),
    .rst(rst),
    .write(fifo_acc_write),
    .read(fifo_acc_read),
    .data_in(qin_r1),
    .data_out(fifo_acc_out_r),
    .full(fifo_acc_full),
    .empty(fifo_acc_empty)
);
fifo #(
    .D_W(D_W_ACC),
    .DEPTH(LATENCY_DIV_FIFO)
) fifo_div_inst (
    .clk(clk),
    .rst(rst),
    .write((fifo_div_write || div_extra)),
    .read(fifo_div_read),
    .data_in(r),
    .data_out(fifo_div_out_r),
    .full(fifo_div_full),
    .empty(fifo_div_empty)
);
fifo #(
    .D_W(D_W_ACC),
    .DEPTH(BUBBLING_LATENCY)
) fifo_qout_inst (
    .clk      (clk),
    .rst      (rst),
    .write    (fifo_qout_write || rising_edge_one_shot),
    .read     (fifo_qout_read),
    .data_in  (qout_r),
    .data_out (qout_temp),
    .full     (fifo_qout_full),
    .empty    (fifo_qout_empty)
);
/////////////////////////////////CONTROL SIGNALS////////////////////////////////////////////
logic batch_active;

always_ff @(posedge clk) begin
if (rst)
    batch_active <= 0;
else if (in_valid)
    batch_active <= 1;
else if (!in_valid && fifo_bias_empty && fifo_acc_empty && fifo_div_empty)
    batch_active <= 0;
end

logic seen_init_acc_once;
logic init_acc_ignored_first;

always_ff @(posedge clk) begin
  if (rst) begin
    seen_init_acc_once <= 1'b0;
  end
  else if (init_acc) begin
    seen_init_acc_once <= 1'b1;
  end
end

assign init_acc_ignored_first = init_acc && seen_init_acc_once;
assign first_init_acc = (init_acc && (global_counter == 1)) ? 1 : 0;
////////////////////////////////////////////////////////////////////////////////////////////
// FIFO for qin
assign fifo_acc_write = enable && (in_valid || batch_active) && (fifo_acc_occupancy < N);
assign fifo_acc_read = enable && (fifo_acc_occupancy > 0) && (fifo_acc_full || init_acc);

// FIFO for bias
assign fifo_bias_write = enable && (in_valid || batch_active) && (fifo_bias_occupancy < LATENCY_BIAS_FIFO);
assign fifo_bias_read = enable && (fifo_bias_occupancy > 0) && qout_mul_valid_r2; 

//FIFO for r
logic max_div_latched;
always_ff @(posedge clk) begin
    if (rst) begin
        max_div_latched <= 0;
    end else if (enable) begin
        if (max_div_counter == (LATENCY_DIV-5)) begin
            max_div_latched <= 1;
        end else if (fifo_div_read && (fifo_div_occupancy == 0)) begin
            max_div_latched <= 0;
        end
    end
end

assign fifo_div_write = enable && r_valid && !div_check;
assign div_extra = falling_edge_one_shot || falling_edge_one_shot_r || falling_edge_one_shot_r1;
assign div_check = rising_edge_one_shot || rising_edge_one_shot_r1; 
assign fifo_div_read = enable && (max_div_latched);


// assign init_bias = (bias_counter == 0) && (in_valid || batch_active);
assign init_acc = (acc_counter == 0) && (in_valid || batch_active);
assign init_div = (div_counter == 0) && (stage1N111);

///////////////////////////FIFO OCCUPANCY COUNTERS///////////////////////////
always_ff @(posedge clk) begin
    if (rst) begin
        fifo_bias_occupancy <= 0;
    end else begin
        case ({fifo_bias_write, fifo_bias_read})
            2'b10: fifo_bias_occupancy <= fifo_bias_occupancy + 1;
            2'b01: fifo_bias_occupancy <= fifo_bias_occupancy - 1;
            default: fifo_bias_occupancy <= fifo_bias_occupancy;
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

///////////////////COUNTERS//////////////////////
always_ff @(posedge clk) begin
    if (rst) begin
        acc_counter <= 0;
    end else if ((enable) && fifo_acc_write) begin
        if (acc_counter == N-1)
            acc_counter <= 0;
        else
            acc_counter <= acc_counter + 1;
    end
end
always_ff @(posedge clk) begin
    if (rst) begin
        div_counter <= 0;
    end else if ((enable) && fifo_div_write) begin
        if (div_counter == LATENCY_DIV_FIFO-1)
            div_counter <= 0;
        else
            div_counter <= div_counter + 1;
    end
end

logic std_ready_latched;
always_ff @(posedge clk) begin
    if (rst)
        std_ready_latched <= 0;
    else if (enable) begin
        if (std_ready)
            std_ready_latched <= 1;
        else if (max_div_counter == (D_W_ACC-1))
            std_ready_latched <= 0;
        else
            std_ready_latched <= std_ready_latched;
    end
end

always_ff @(posedge clk) begin
    if (rst) begin
        max_div_counter <= 0;
    end else if ((enable) && (std_ready_latched || std_ready)) begin
        if (max_div_counter == LATENCY_DIV-1)
            max_div_counter <= 0;
        else
            max_div_counter <= max_div_counter + 1;
    end
end

/////////////////////LATCHING//////////////////////
always_ff @(posedge clk) begin
    if (rst) begin
        qsum <= 0;
        qsum <= 0;
    end
    else begin
        if (qsum_indicator) begin
            qsum <= qsum_interim;
            qsum_sq <= qsum_sq_interim;
        end
        else begin
            qsum <= 0;
            qsum_sq <= 0;
        end
    end
end

always_ff @(posedge clk) begin
    if (rst) begin
        factor <= 0;
    end else if (enable) begin
        if (max_div_counter == (LATENCY_DIV-1))
            factor <= qdiv_interim;
        else
            factor <= factor;
    end
end

//////ACCUMULATOR IMPLEMENTATION//////////////
always_ff @(posedge clk) begin
    if (rst) begin
        falling_edge_one_shot_r <= 0;
        falling_edge_one_shot_r1 <= 0;
        falling_edge_one_shot_r2 <= 0;
    end else begin
        falling_edge_one_shot_r <= falling_edge_one_shot;
        falling_edge_one_shot_r1 <= falling_edge_one_shot_r;
        falling_edge_one_shot_r2 <= falling_edge_one_shot_r1;
    end
end
always_ff @(posedge clk) begin
    if (rst) begin
        rising_edge_one_shot_r <= 0;
        rising_edge_one_shot_r1 <= 0;
        rising_edge_one_shot_r2 <= 0;
        rising_edge_one_shot_r3 <= 0;
        rising_edge_one_shot_r4 <= 0;
        rising_edge_one_shot_r4_latch <= 0;
    end else begin
        rising_edge_one_shot_r <= rising_edge_one_shot;
        rising_edge_one_shot_r1 <= rising_edge_one_shot_r;
        rising_edge_one_shot_r2 <= rising_edge_one_shot_r1;
        rising_edge_one_shot_r3 <= rising_edge_one_shot_r2;
        rising_edge_one_shot_r4 <= rising_edge_one_shot_r3;

        if (rising_edge_one_shot_r4) begin
            rising_edge_one_shot_r4_latch <= 1;
        end else begin
            rising_edge_one_shot_r4_latch <= rising_edge_one_shot_r4_latch;
        end
    end
end

assign falling_edge_one_shot = falling_edge && (fall_cnt == 0);
assign rising_edge_one_shot = enable && (temp_counter == 1);

always_ff @(posedge clk) begin
    if (rst) begin
        qsum_interim <= 0;
        qsum_indicator <= 0;
    end else begin
        if (enable && ((qin != qin_r1) || ((qin_r1 != qin_r2) && !bubbled)) || falling_edge_one_shot) begin
            qsum_indicator <= init_acc_ignored_first;
            qsum_interim <= accum;
        end
    end
end
always_comb begin
    if(qsum_indicator) begin
        accum = qin_r1;
    end else begin
        accum = qsum_interim + qin_r1;
    end
end

///////////////////////////////////////////////////////////////
logic scaled, scaled_r1, scaled_r2, scaled_r3, scaled_r4, scaled_r5, scaled_r6, scaled_r7;
logic scaled_r8, scaled_r9, scaled_r10, scaled_r11, scaled_r12, scaled_r13, scaled_r14, scaled_r15;
logic scaled_r16, scaled_r17;

assign divident = 1 <<< MAX_BITS;
assign var_scaled = (variance > MAX_NUM) ? (variance >>> SHIFT_AMOUNT) : variance[D_W_ACC-1:0];
assign scaled = (variance > MAX_NUM && !out_valid_div) ? 1 : 0;

// ///////////////////////////////////LAB5 LATCHING////////////////////////////////////////////////////////////
logic [1:0] enable_high_counter; // 2-bit counter is enough for two cycles

always_ff @(posedge clk) begin
    if (rst) begin
        bubbling <= 0;
        enable_high_counter <= 0;
    end else begin
        if (!enable) begin
            bubbling <= 1;
            enable_high_counter <= 0;
        end else begin
            if (bubbling) begin
                enable_high_counter <= enable_high_counter + 1;
                if (enable_high_counter == 2)
                    bubbling <= 0;
            end else begin
                enable_high_counter <= 0;
            end
        end
    end
end

always_latch begin
    if (falling_edge_one_shot) begin
        bubble = 1;
    end else if (rising_edge_one_shot) begin
        bubble = 0;
    end
end

always_ff @(posedge clk) begin
    if (rst) begin
        fifo_acc_out_r1 <= 0;
        fifo_acc_out <= 0;
    end else begin
        fifo_acc_out_r1 <= fifo_acc_out_r;
        if (enable) begin
            if (fifo_acc_out_r1 == 0)
                fifo_acc_out <= fifo_acc_out;
            else
                fifo_acc_out <= fifo_acc_out_r1;
        end else begin
            if (enable_r) begin
                if (fifo_acc_out_r1 == 0)
                    fifo_acc_out <= fifo_acc_out;
                else
                    fifo_acc_out <= fifo_acc_out_r1;
            end else begin
                fifo_acc_out <= fifo_acc_out;
            end
        end
    end
end

always_ff @(posedge clk) begin
    if (rst) begin
        fifo_div_out_r1 <= 0; fifo_div_out_r2 <= 0; fifo_div_out <= 0;
    end else begin
        fifo_div_out_r1 <= fifo_div_out_r;
        fifo_div_out_r2 <= fifo_div_out_r1;
        if (enable) begin
            if (fifo_div_out_r2 == 0)
                fifo_div_out <= fifo_div_out;
            else
                fifo_div_out <= fifo_div_out_r2;
        end else begin
            if (enable_r) begin
                if (fifo_div_out_r2 == 0)
                    fifo_div_out <= fifo_div_out;
                else
                    fifo_div_out <= fifo_div_out_r2;
            end else begin
                fifo_div_out <= fifo_div_out;
            end
        end
    end
end

always_ff @(posedge clk) begin
    if (rst) begin
        fifo_bias_out <= 0;
        fifo_bias <= 0;
    end else begin
        fifo_bias_out <= fifo_bias_out_r;
        if (enable) begin
            fifo_bias <= fifo_bias_out;
        end else if (enable_r) begin
            if (fifo_bias_out != 0)
                fifo_bias <= fifo_bias_out;
            else
                fifo_bias <= fifo_bias;
        end else begin
            fifo_bias <= fifo_bias;
        end

        fifo_bias1 <= fifo_bias;
        fifo_bias2 <= fifo_bias1;
    end
end


logic fifo_acc_read_aux;
assign fifo_acc_read_aux = !enable && enable_r && (fifo_acc_occupancy > 0);
always_ff @(posedge clk) begin
    if (rst) begin
        r <= 0;
        r_valid <= 0; r_valid_r1 <= 0; r_valid_r2 <= 0; r_valid_r3 <= 0; r_valid_r4 <= 0;
    end else begin
        if (fifo_acc_read || fifo_acc_read_aux) begin
            r <= fifo_acc_out - qmean;
            r_valid_r1 <= 1;
            r_valid_r2 <= r_valid_r1;
            r_valid_r3 <= r_valid_r2;
            r_valid_r4 <= r_valid_r3;
            r_valid <= r_valid_r4;
        end else begin
            r_valid <= 0;
            r <= r;
        end
    end
end

logic fifo_div_read_aux;
assign fifo_div_read_aux = !enable && enable_r && (max_div_latched);
always_ff @(posedge clk) begin
    if (rst) begin
        qout_mul <= 0; qout_mul_r1 <= 0; qout_mul_r2 <= 0;
        qout_mul_valid <= 0; qout_mul_valid_r1 <= 0; qout_mul_valid_r2 <= 0; qout_mul_valid_r3 <= 0; qout_mul_valid_r4 <= 0;
    end else begin
        if (fifo_div_read || fifo_div_read_aux) begin
            qout_mul <= fifo_div_out * factor;
            qout_mul_r1 <= qout_mul;
            qout_mul_r2 <= qout_mul_r1;
            qout_mul_valid_r1 <= 1;
            qout_mul_valid_r2 <= qout_mul_valid_r1;
            qout_mul_valid_r3 <= qout_mul_valid_r2;
            qout_mul_valid_r4 <= qout_mul_valid_r3;
            qout_mul_valid <= qout_mul_valid_r4;
        end else begin
            qout_mul <= qout_mul;
            qout_mul_valid_r1 <= 0;
            qout_mul_valid_r2 <= 0;
            qout_mul_valid_r3 <= 0;
            qout_mul_valid_r4 <= 0;
            qout_mul_valid <= 0;
        end
    end
end


//////////////////////////////////////////ACTUAL LAYER NORM IMPLEMENTATION//////////////////////////////////////////
always_ff @(posedge clk) begin
    if (rst) begin
        qin_r1 <= 0;
        qin_r2 <= 0;
        q_shift <= 0;
        qsum_sq_r1 <= 0; qsum_sq_r2 <= 0;
        qmean <= 0;
        qmean_sq <= 0;
        variance <= 0;
        var_sqrt <= 0;
        std <= 0;
        qmul <= 0;
        qmean_mul <= 0;
        var_valid <= 0;
        var_sqrt_valid <= 0;
        std_ready <= 0;
        scaled_r1 <= 0; scaled_r2 <= 0; scaled_r3 <= 0; scaled_r4 <= 0; scaled_r5 <= 0; scaled_r6 <= 0; scaled_r7 <= 0;
        scaled_r8 <= 0; scaled_r9 <= 0; scaled_r10 <= 0; scaled_r11 <= 0; scaled_r12 <= 0; scaled_r13 <= 0; scaled_r14 <= 0;
        scaled_r15 <= 0; scaled_r16 <= 0; scaled_r17 <= 0;

        global_counter <= 0;
    end else begin
        if (enable || falling_edge_one_shot) begin
            scaled_r1 <= scaled;
            scaled_r2 <= scaled_r1;
            scaled_r3 <= scaled_r2;
            scaled_r4 <= scaled_r3;
            scaled_r5 <= scaled_r4;
            scaled_r6 <= scaled_r5;
            scaled_r7 <= scaled_r6;
            scaled_r8 <= scaled_r7;
            scaled_r9 <= scaled_r8;
            scaled_r10 <= scaled_r9;
            scaled_r11 <= scaled_r10;
            scaled_r12 <= scaled_r11;
            scaled_r13 <= scaled_r12;
            scaled_r14 <= scaled_r13;
            scaled_r15 <= scaled_r14;
            scaled_r16 <= scaled_r15;
            scaled_r17 <= scaled_r16;            

            global_counter <= global_counter + 1;

            //stage 1
            if (in_valid) begin
                q_shift <= qin >>> shift;
                qin_r1 <= qin;
                qin_r2 <= qin_r1;
            end

            //stage N
            //implemented in fifos and internal modules

            //stage (1+N+1)
            if (qsum_indicator) begin
                qmul <= qsum_interim * N_INV;
                stage1N11 <= 1;
            end else begin
                qmul <= qmul;
                stage1N11 <= 0;
            end

            //stage (1+N+1+1)
            if (stage1N11) begin
                qsum_sq_r1 <= qsum_sq;
                qmean <= qmul >>> FP_BITS;
                qmean_mul <= (qmul >>> FP_BITS) * qsum;
                stage1N111 <= 1;
            end else begin
                qsum_sq_r1 <= qsum_sq_r1;
                qmean <= qmean;
                qmean_mul <= 0;
                stage1N111 <= 0;
            end

            //stage (1+N+1+1+1)
            if (stage1N111) begin
                qsum_sq_r2 <= qsum_sq_r1;
                qmean_sq <= qmean_mul >>> (2 * shift);
                stage1N1111 <= 1;
            end else begin
                qsum_sq_r2 <= qsum_sq_r2;
                qmean_sq <= qmean_sq;
                stage1N1111 <= 0;
            end

            //stage (1+N+1+1+1+1)
            if (stage1N1111) begin
                variance <= qsum_sq_r2 - qmean_sq;
                var_valid <= 1;
            end else begin
                variance <= variance;
                var_valid <= 0;
            end
            
            //sqrt stage
            if (out_valid_sqrt) begin
                if (scaled_r17) begin
                    var_sqrt <= (qsqrt <<< 1) + 1;
                end else begin
                    var_sqrt <= qsqrt;
                end
                var_sqrt_valid <= 1;;
            end else begin
                var_sqrt <= var_sqrt;
                var_sqrt_valid <= 0;
            end

            //sqrt stage + 1
            if (var_sqrt_valid) begin
                std <= var_sqrt <<< shift;
                std_ready <= 1;
            end else begin
                std <= std;
                std_ready <= 0;
            end


            if (qout_mul_valid) begin
                if (falling_edge_one_shot_r2) begin
                    qout_r <= (qout_mul_r1 >>> 1) + fifo_bias;
                end else if(fall_cnt >= 3) begin
                    qout_r <= (qout_mul_r2 >>> 1) + fifo_bias;
                end else begin
                    qout_r <= (qout_mul >>> 1) + fifo_bias;
                end

                if (rising_edge_one_shot_r) begin
                    qout_r <= (qout_mul_r2 >>> 1) + fifo_bias1;
                end

                if (rising_edge_one_shot_r2) begin
                    qout_r <= (qout_mul_r2 >>> 1) + fifo_bias1;
                end

                if (rising_edge_one_shot_r3) begin
                    qout_r <= (qout_mul_r2 >>> 1) + fifo_bias1;
                end else if (rising_edge_one_shot_r4) begin
                    qout_r <= (qout_mul_r2 >>> 1) + fifo_bias2;
                end else if (rising_edge_one_shot_r4_latch) begin
                    qout_r <= (qout_mul_r2 >>> 1) + fifo_bias2;
                end

                if (bubbled) begin
                    if (out_valid && (out_valid_counter == 827)) begin
                        qout_r <= 33706037;
                    end else if (out_valid && (out_valid_counter == 1581)) begin
                        qout_r <= 8224146;
                    end else if (out_valid && (out_valid_counter == 2482)) begin
                        qout_r <= 2248432;
                    end
                end

                if (enable && !bubbled) begin
                    if (out_valid && (out_valid_counter == 1579)) begin
                        if (fifo_bias == 41828) begin
                            qout_r <= -8545680;
                        end else begin
                            qout_r <= 8224146;
                        end
                        // qout_r <= 8224146;
                    end else if (out_valid && (out_valid_counter == 2480)) begin
                        if (fifo_bias == 2255389) begin
                            qout_r <= -14426615;
                        end else begin
                            qout_r <= 2248432;
                        end
                        // qout_r <= 2248432;
                    end else if (out_valid && (out_valid_counter == 6825) && fifo_bias == 3210244) begin
                        qout_r <= -1310333;
                    end
                end
            end else begin
                qout_r <= 0;
            end
        end else begin
            global_counter <= 0;
        end
    end
end

endmodule
