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
    wire [DATA_WIDTH-1:0] p00 = window_data[143:128];
    wire [DATA_WIDTH-1:0] p01 = window_data[127:112];
    wire [DATA_WIDTH-1:0] p02 = window_data[111:96];
    wire [DATA_WIDTH-1:0] p10 = window_data[95:80];
    wire [DATA_WIDTH-1:0] p11 = window_data[79:64];
    wire [DATA_WIDTH-1:0] p12 = window_data[63:48];
    wire [DATA_WIDTH-1:0] p20 = window_data[47:32];
    wire [DATA_WIDTH-1:0] p21 = window_data[31:16];
    wire [DATA_WIDTH-1:0] p22 = window_data[15:0];

    wire any_nb = (p00 > dynamic_threshold) || (p01 > dynamic_threshold) || 
                  (p02 > dynamic_threshold) || (p10 > dynamic_threshold) || 
                  (p12 > dynamic_threshold) || (p20 > dynamic_threshold) || 
                  (p21 > dynamic_threshold) || (p22 > dynamic_threshold);
    
    reg [9:0] x_cnt, y_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // [PATCH] The first valid 3x3 center pixel is physically located at X=2, Y=1
            x_cnt <= 10'd2; 
            y_cnt <= 10'd1;
            f_pixel <= 0; 
            f_valid <= 0;
        end else if (valid_in) begin
            // Coordinate Tracking
            if (x_cnt == IMAGE_WIDTH-1) begin 
                x_cnt <= 0; 
                y_cnt <= y_cnt + 1; 
            end else begin
                x_cnt <= x_cnt + 1;
            end

            // Threshold + Hot Pixel Masking
            if (p11 > dynamic_threshold && any_nb) begin
                f_pixel <= p11 - dynamic_threshold;
            end else begin
                f_pixel <= 0;
            end
            
            f_x <= x_cnt; 
            f_y <= y_cnt;
            f_valid <= 1;
        end else begin
            f_valid <= 0;
        end
    end
endmodule