`timescale 1ns / 1ps

module line_delay #(
    parameter DATA_WIDTH = 16,
    parameter IMAGE_WIDTH = 1024
)(
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire [DATA_WIDTH-1:0] din,
    input  wire                  valid_in,
    output wire [DATA_WIDTH-1:0] dout
);

    // Infer Block RAM (BRAM) for line storage
    reg [DATA_WIDTH-1:0] ram [0:IMAGE_WIDTH-1];
    reg [9:0] ptr; // 10-bit pointer for 1024 addresses
    
    // Read/Write logic
    always @(posedge clk) begin
        if (valid_in) begin
            ram[ptr] <= din;
        end
    end
    
    // Pointer management
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ptr <= 0;
        end else if (valid_in) begin
            if (ptr == IMAGE_WIDTH-1) ptr <= 0;
            else ptr <= ptr + 1;
        end
    end
    
    assign dout = ram[ptr];

endmodule