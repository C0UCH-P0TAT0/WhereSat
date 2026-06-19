// fpga/rtl/threshold_filter.v
module threshold_filter #(
    parameter DATA_WIDTH = 16,
    parameter IMAGE_WIDTH = 1024
)(
    input  wire                   clk,
    input  wire                   rst_n,
    input  wire [DATA_WIDTH*9-1:0] window_data,
    input  wire                   valid_in,
    input  wire [DATA_WIDTH-1:0]  dynamic_threshold,
    output reg [DATA_WIDTH-1:0]   f_pixel,
    output reg [9:0]              f_x, f_y,
    output reg                    f_valid
);
    wire [DATA_WIDTH-1:0] p11 = window_data[79:64];
    wire any_nb = (window_data[143:80] > dynamic_threshold) || (window_data[63:0] > dynamic_threshold);
    
    reg [9:0] x_cnt, y_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            {x_cnt, y_cnt, f_pixel, f_valid} <= 0;
        end else if (valid_in) begin
            // Coordinate Tracking
            if (x_cnt == IMAGE_WIDTH-1) begin x_cnt <= 0; y_cnt <= y_cnt + 1; end
            else x_cnt <= x_cnt + 1;

            // Threshold + Hot Pixel
            if (p11 > dynamic_threshold && any_nb) f_pixel <= p11 - dynamic_threshold;
            else f_pixel <= 0;
            
            f_x <= x_cnt; f_y <= y_cnt;
            f_valid <= 1;
        end else f_valid <= 0;
    end
endmodule