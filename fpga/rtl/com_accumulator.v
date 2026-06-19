// fpga/rtl/com_accumulator.v
module com_accumulator (
    input  wire        clk, rst_n,
    input  wire [15:0] pixel_intensity,
    input  wire [9:0]  x_coord, y_coord,
    input  wire        blob_active,
    input  wire        blob_done,
    output reg [23:0]  centroid_x, centroid_y, // Q16.8
    output reg         centroid_valid
);
    reg [23:0] m00;
    reg [39:0] m10, m01;
    
    // Sequential Divider Signals
    reg [47:0] quot_reg;
    reg [47:0] accum_reg;
    reg [23:0] divisor_reg;
    reg [5:0]  bit_cnt;
    reg [1:0]  state;

    localparam IDLE=0, DIV_X=1, DIV_Y=2, DONE=3;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            {m00, m10, m01, centroid_valid} <= 0;
        end else begin
            case (state)
                IDLE: begin
                    centroid_valid <= 0;
                    if (blob_active) begin
                        m00 <= m00 + pixel_intensity;
                        m10 <= m10 + (pixel_intensity * x_coord);
                        m01 <= m01 + (pixel_intensity * y_coord);
                    end
                    if (blob_done && m00 > 0) begin
                        state <= DIV_X;
                        bit_cnt <= 0;
                        accum_reg <= {m10, 8'b0}; // Q16.8 shift
                        divisor_reg <= m00;
                    end
                end

                DIV_X, DIV_Y: begin
                    // Sequential Shift-and-Subtract Divider
                    if (bit_cnt < 40) begin
                        if (accum_reg >= (divisor_reg << (39-bit_cnt))) begin
                            accum_reg <= accum_reg - (divisor_reg << (39-bit_cnt));
                            quot_reg[39-bit_cnt] <= 1;
                        end else begin
                            quot_reg[39-bit_cnt] <= 0;
                        end
                        bit_cnt <= bit_cnt + 1;
                    end else begin
                        if (state == DIV_X) begin
                            centroid_x <= quot_reg[23:0];
                            state <= DIV_Y;
                            bit_cnt <= 0;
                            accum_reg <= {m01, 8'b0};
                        end else begin
                            centroid_y <= quot_reg[23:0];
                            state <= DONE;
                        end
                    end
                end

                DONE: begin
                    centroid_valid <= 1;
                    {m00, m10, m01} <= 0;
                    state <= IDLE;
                end
            endcase
        end
    end
endmodule