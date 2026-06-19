// fpga/rtl/line_buffer.v
module line_buffer #(
    parameter DATA_WIDTH = 16,
    parameter IMAGE_WIDTH = 1024
)(
    input  wire                   clk,
    input  wire                   rst_n,
    input  wire [DATA_WIDTH-1:0]  pixel_in,
    input  wire                   valid_in,
    output wire [DATA_WIDTH*9-1:0] window_data,
    output reg                    valid_out
);
    wire [DATA_WIDTH-1:0] line0_out, line1_out;
    reg [DATA_WIDTH-1:0] win [0:2][0:2];
    
    // Pipeline Priming: 2 lines + 2 pixels = 2050 cycles
    reg [11:0] startup_cnt; 
    reg        primed;

    line_delay #(DATA_WIDTH, IMAGE_WIDTH) rd0 (clk, rst_n, pixel_in, valid_in, line0_out);
    line_delay #(DATA_WIDTH, IMAGE_WIDTH) rd1 (clk, rst_n, line0_out, valid_in, line1_out);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            startup_cnt <= 0;
            primed <= 0;
            valid_out <= 0;
        end else if (valid_in) begin
            // Shift window
            win[2][0] <= pixel_in; win[2][1] <= win[2][0]; win[2][2] <= win[2][1];
            win[1][0] <= line0_out; win[1][1] <= win[1][0]; win[1][2] <= win[1][1];
            win[0][0] <= line1_out; win[0][1] <= win[0][0]; win[0][2] <= win[0][1];

            if (!primed) begin
                if (startup_cnt >= (IMAGE_WIDTH*2 + 2)) primed <= 1;
                else startup_cnt <= startup_cnt + 1;
            end
            valid_out <= primed;
        end else valid_out <= 0;
    end

    assign window_data = {win[0][0], win[0][1], win[0][2], win[1][0], win[1][1], win[1][2], win[2][0], win[2][1], win[2][2]};
endmodule