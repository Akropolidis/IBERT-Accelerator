`timescale 1ps / 1ps

module mem_read_D
#(
    parameter integer N1           = 4,
    parameter integer N2           = 4,
    parameter integer MATRIXSIZE_W = 16,
    parameter integer ADDR_W       = 12
)
(
    input  logic                    clk,
    input  logic                    rst,
    input  logic [MATRIXSIZE_W-1:0] M3,
    input  logic [MATRIXSIZE_W-1:0] M1dN1,
    input  logic                    valid_D,
    output logic  [ADDR_W-1:0]       rd_addr_D,
    output logic  [N1-1:0]           activate_D
);

logic [N1-1:0]           activate_D_r;
logic [MATRIXSIZE_W-1:0] col;
logic [$clog2(N1)-1:0]   sys_row;
logic [MATRIXSIZE_W-1:0] offset;
logic [MATRIXSIZE_W-1:0] phase;
logic [$clog2(N2)-1:0]   mini_col;
logic [MATRIXSIZE_W-1:0] mini_offset;

always_ff @(posedge clk) begin
    if (rst) begin 
        col         <= 0;
        sys_row     <= 0;
        offset      <= 0;
        phase       <= 0;
        mini_col    <= 0;
        mini_offset <= 0;
        rd_addr_D   <= 0;
    end else if (valid_D) begin
        col <= col + 1;
        mini_col <= mini_col + 1;
        
        if (mini_col == N2-1) begin
            mini_col    <= 0;
            mini_offset <= mini_offset + N2;
        end
        
        if (col == M3-1) begin
            col <= 0;
            mini_offset <= 0;
            sys_row <= sys_row + 1;
            if (sys_row == N1-1) begin
                offset  <= offset + M3;
                phase   <= phase + 1;
                sys_row <= 0;
            end
        end
        
        rd_addr_D <= (N2 - mini_col - 1) + mini_offset + offset;
    end
end

integer x;
always_ff @(posedge clk) begin
    if (rst) begin
        activate_D_r <= 1;      // [0,0,...,1]
        activate_D   <= 0;
    end else begin
        activate_D <= activate_D_r;
        if (valid_D) begin
            if (col == M3-1) begin
                activate_D_r[0] <= (phase == M1dN1-1) ? 0 : activate_D_r[N1-1];
                for (x = 1; x < N1; x = x + 1) begin
                    activate_D_r[x] <= activate_D_r[x-1];
                end
            end
        end
    end
end

// logic [MATRIXSIZE_W-1:0] counter_D;
// logic [N1-1:0] bank_D;
// logic [MATRIXSIZE_W-1:0] temp;
// logic [MATRIXSIZE_W-1:0] addr_accumulator;

// always_ff @(posedge clk) begin
//     if (rst) begin
//         counter_D <= M3 - 1;
//         bank_D <= 0;
//         temp <= 0;
//         addr_accumulator <= 0;
//     end else begin
//         if (valid_D) begin
//             if (counter_D == 0) begin
//                 counter_D <= M3 - 1;
//                 if (bank_D == (N1 - 1)) begin
//                     bank_D <= 0;
//                     temp <= temp + 1; //end of bank, move to next bank
//                     addr_accumulator <= addr_accumulator + M3;
//                 end else begin
//                     bank_D <= bank_D + 1;
//                 end
//             end else begin
//                 counter_D <= counter_D - 1;
//             end
//         end
//     end
// end

// always_ff @(posedge clk) begin
//     if (rst) begin
//         rd_addr_D <= 0;
//         activate_D <= 0;
//     end else begin
//         if (valid_D && temp < M1dN1) begin
//             if (counter_D >= N2) begin
//                 rd_addr_D <= addr_accumulator + (counter_D - N2);
//             end else begin
//                 rd_addr_D <= addr_accumulator + (counter_D + (M3 - N2));
//             end
            
//             activate_D <= 0; //reset all banks before setting the next bank
//             activate_D[bank_D] <= 1;
//         end
//     end
// end

endmodule
