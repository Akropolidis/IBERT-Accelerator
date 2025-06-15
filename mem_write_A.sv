`timescale 1ps / 1ps

module mem_write_A
#(
    parameter integer N1           = 4,
    parameter integer MATRIXSIZE_W = 16,
    parameter integer ADDR_W       = 12
)
(
    input  logic                    clk,
    input  logic                    rst,
    input  logic [MATRIXSIZE_W-1:0] M2,
    input  logic [MATRIXSIZE_W-1:0] M1dN1,
    input  logic                    valid_A,
    output logic  [ADDR_W-1:0]       wr_addr_A,
    output logic  [N1-1:0]           activate_A
);

// logic [MATRIXSIZE_W-1:0] col;
// logic [$clog2(N1)-1:0]   sys_row;
// logic [MATRIXSIZE_W-1:0] offset;
// logic [MATRIXSIZE_W-1:0] phase;
// logic [N1-1:0]           activate_A_r;

// always_ff @(posedge clk) begin
//     if (rst) begin
//         col     <= 0;
//         sys_row <= 0;
//         offset  <= 0;
//         phase   <= 0;
//         wr_addr_A <= 0;
//     end else if (valid_A) begin
//         col <= col + 1;
//         if (col == M2-1) begin
//             col <= 0;
//             sys_row <= sys_row + 1;
//             if (sys_row == N1-1) begin
//                 sys_row <= 0;
//                 offset  <= offset + M2;
//                 phase   <= phase + 1;
//             end
//         end
//         wr_addr_A <= col + offset;
//     end
// end

// integer x;
// always_ff @(posedge clk) begin
//     if (rst) begin
//         activate_A_r <= 1;    // [0,0,...,1]
//         activate_A   <= 0;
//     end else begin
//         activate_A <= activate_A_r; 
//         if (valid_A) begin
//             if (col == M2-1) begin
//                 activate_A_r[0] <= (phase == M1dN1-1) ? 0 : activate_A_r[N1-1];
//                 for (x = 1; x < N1; x = x + 1) begin
//                     activate_A_r[x] <= activate_A_r[x-1];
//                 end
//             end
//         end
//     end
// end

logic [MATRIXSIZE_W-1:0] counter_A;
logic [N1-1:0] bank_A;
logic [MATRIXSIZE_W-1:0] temp;
logic [MATRIXSIZE_W-1:0] addr_accumulator;

always_ff @(posedge clk) begin
    if (rst) begin
        counter_A <= 0;
        bank_A <= 0;
        temp <= 0;
    end else begin
        if (valid_A) begin
            if (counter_A == (M2 - 1)) begin
                counter_A <= 0;
                if (bank_A == (N1 - 1)) begin
                    bank_A <= 0;
                    temp <= temp + 1; //end of bank, move to next bank
                end else begin
                    bank_A <= bank_A + 1;
                end
            end else begin
                counter_A <= counter_A + 1;
            end
        end
    end
end

always_ff @(posedge clk) begin
    if (rst) begin
        wr_addr_A <= 0;
        activate_A <= 0;
    end else begin
        if (valid_A) begin
            if (temp < M1dN1) begin
                wr_addr_A <= addr_accumulator + counter_A;
            end
            activate_A <= 0; //reset all banks before setting the next bank
            activate_A[bank_A] <= 1;
        end
    end
end

always_ff @(posedge clk) begin
    if (rst) begin
        addr_accumulator <= 0;
    end else begin
        if (valid_A) begin
            if (counter_A == (M2 - 1)) begin
                if (bank_A == (N1 - 1)) begin //at end of first bank, moving to next bank
                    addr_accumulator <= addr_accumulator + M2;
                end
            end
        end
    end
end

endmodule
