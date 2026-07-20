`timescale 1ns / 1ps

// WhereSat Project: FPGA Vision Pipeline
// Module: threshold_filter (Production Version)
// Description: Pedestal subtraction and Sparse Event Bus generator. 
// Stripped NMS to preserve physical star halos for Q16.8 sub-pixel interpolation.

module threshold_filter #(
    parameter DATA_WIDTH = 16,
    parameter IMAGE_WIDTH = 1024
)(
    input  wire                   clk,
    input  wire                   rst_n,
    input  wire [DATA_WIDTH*9-1:0] window_data,
    input  wire                   valid_in,
    input  wire [DATA_WIDTH-1:0]  dynamic_threshold,
    input  wire                   frame_done, // Explicit VSYNC Reset

    output reg  [DATA_WIDTH-1:0]  f_pixel,
    output reg  [9:0]             f_x,
    output reg  [9:0]             f_y,
    output reg                    f_valid
);

    wire [DATA_WIDTH-1:0] p11 = window_data[79:64];

    // Clean combinatorial assignments for the event gate
    wire event_trigger = (p11 > dynamic_threshold);
    wire [DATA_WIDTH-1:0] signal = p11 - dynamic_threshold;

    reg [9:0] x_cnt;
    reg [9:0] y_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            
            // due to the 2-row + 2-pixel latency of the upstream line_buffer.
            x_cnt    <= 10'd2;
            y_cnt    <= 10'd1;
            f_pixel  <= 0;
            f_x      <= 0;
            f_y      <= 0;
            f_valid  <= 0;
        end else begin
            // 1. Explicit Frame Reset
            if (frame_done) begin
                x_cnt    <= 10'd2;
                y_cnt    <= 10'd1;
                f_pixel  <= 0;
                f_x      <= 0; 
                f_y      <= 0; 
                f_valid  <= 0;
            end 
            // 2. Stream Processing
            else if (valid_in) begin
                f_x <= x_cnt;
                f_y <= y_cnt;

                // 3. The Sparse Event Gate & Pedestal Subtraction
                f_valid <= event_trigger;
                
                if (event_trigger) begin
                    f_pixel <= signal;
                end else begin
                    f_pixel <= 0;
                end

                // 4. Coordinate tracking with strict boundaries
                if (x_cnt == (IMAGE_WIDTH - 1)) begin
                    x_cnt <= 10'd0;
                    if (y_cnt != 10'd1023) y_cnt <= y_cnt + 10'd1;
                end else begin
                    x_cnt <= x_cnt + 10'd1;
                end
            end 
            // 3. Pipeline Stall
            else begin
                f_valid <= 1'b0;
            end
        end
    end

endmodule