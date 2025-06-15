`timescale 1ps / 1ps

module mem_write_B
#(
    parameter integer N2           = 4,
    parameter integer MATRIXSIZE_W = 16,
    parameter integer ADDR_W       = 12
)
(
    input  logic                    clk,
    input  logic                    rst,
    input  logic [MATRIXSIZE_W-1:0] M2,
    input  logic [MATRIXSIZE_W-1:0] M3dN2,
    input  logic                    valid_B,
    output logic  [ADDR_W-1:0]       wr_addr_B,
    output logic  [N2-1:0]           activate_B
);

// logic [N2-1:0]           activate_B_r;
// logic [$clog2(N2)-1:0]   col;
// logic [MATRIXSIZE_W-1:0] row;
// logic [MATRIXSIZE_W-1:0] offset;
// logic [MATRIXSIZE_W-1:0] phase;

// always_ff @(posedge clk) begin
//     if (rst) begin
//         col       <= 0;
//         row       <= 0;
//         offset    <= 0;
//         phase     <= 0;
//         wr_addr_B <= 0;
//     end else if (valid_B) begin
//         col <= col + 1;
//         if (col == N2-1) begin
//             col <= 0;
//             if (phase == M3dN2-1) begin
//                 offset <= 0;
//                 phase  <= 0;
//                 row    <= row + 1;
//             end else begin
//                 offset <= offset + M2;
//                 phase  <= phase + 1;
//             end
//         end
//         wr_addr_B <= row + offset;
//     end
// end

// integer x;
// always_ff @(posedge clk) begin
//     if (rst) begin
//         activate_B_r <= 1;      // [0,0,...,1]
//         activate_B   <= 0;
//     end else begin
//         activate_B <= activate_B_r;
//         if (valid_B) begin
//             activate_B_r[0] <= (row == M2-1) && (phase == M3dN2-1) ? 0 : activate_B_r[N2-1];
//             for (x = 1; x < N2; x = x + 1) begin
//                 activate_B_r[x] <= activate_B_r[x-1];
//             end
//         end
//     end
// end

logic [MATRIXSIZE_W-1:0] counter_B;
logic [N2-1:0] bank_B;
logic [MATRIXSIZE_W-1:0] temp;
logic [MATRIXSIZE_W-1:0] row_index;
logic [MATRIXSIZE_W-1:0] addr_accumulator;

always_ff @(posedge clk) begin
    if (rst) begin
        counter_B <= 0;
        bank_B <= 0;
        temp <= 0;
        row_index <= 0;
    end else begin
        if (valid_B) begin
            if (bank_B == (N2 - 1)) begin
                bank_B <= 0;
                if (temp == (M3dN2 - 1)) begin
                    temp <= 0;
                end else begin
                    temp <= temp + 1; //end of bank, move to next bank
                end
            end else begin
                bank_B <= bank_B + 1;
            end
            
            if (counter_B == (M3dN2 * N2 - 1)) begin //if counter_B is out of index, reset counter_B
                counter_B <= 0;
                row_index <= row_index + 1;
            end else begin
                counter_B <= counter_B + 1;
            end
        end
    end
end

always_ff @(posedge clk) begin
    if (rst) begin
        wr_addr_B <= 0;
        activate_B <= 0;
    end else begin
        if (valid_B) begin
            wr_addr_B <= row_index + addr_accumulator;
            activate_B <= 0; //reset all banks before setting the next bank
            activate_B[bank_B] <= 1;
        end
    end
end

always_ff @(posedge clk) begin
    if (rst) begin
        addr_accumulator <= 0;
    end else begin
        if (valid_B) begin
            if (bank_B == (N2 - 1)) begin
                if (temp == (M3dN2 - 1)) begin
                    addr_accumulator <= 0;
                end else begin
                    addr_accumulator <= addr_accumulator + M2;
                end
            end
        end
    end
end

endmodule
