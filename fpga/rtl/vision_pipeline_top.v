`timescale 1ns / 1ps

module vision_pipeline_top #(
    parameter DATA_WIDTH = 16,
    parameter IMAGE_WIDTH = 1024,
    // Baseline noise threshold. If Yash's Python script injects heavy noise, raise this.
    parameter THRESHOLD = 16'd100 
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire [15:0] pixel_in,
    input  wire        valid_in,
    input  wire        frame_done,
    
    output wire [23:0] centroid_x,
    output wire [23:0] centroid_y,
    output wire        centroid_valid,
    input  wire        centroid_ready
);

    // Internal Interconnect Wires
    wire [DATA_WIDTH*9-1:0] window_data;
    wire                    window_valid;

    wire [DATA_WIDTH-1:0]   f_pixel;
    wire [9:0]              f_x;
    wire [9:0]              f_y;
    wire                    f_valid;

    // 1. Ingest: The Streaming Line Buffer
    line_buffer #(
        .DATA_WIDTH(DATA_WIDTH),
        .IMAGE_WIDTH(IMAGE_WIDTH)
    ) u_line_buffer (
        .clk(clk),
        .rst_n(rst_n),
        .pixel_in(pixel_in),
        .valid_in(valid_in),
        .window_data(window_data),
        .valid_out(window_valid)
    );

    // 2. Clean: The Hardware Guillotine
    threshold_filter #(
        .DATA_WIDTH(DATA_WIDTH),
        .IMAGE_WIDTH(IMAGE_WIDTH)
    ) u_threshold_filter (
        .clk(clk),
        .rst_n(rst_n),
        .window_data(window_data),
        .valid_in(window_valid),
        .dynamic_threshold(THRESHOLD), 
        .f_pixel(f_pixel),
        .f_x(f_x),
        .f_y(f_y),
        .f_valid(f_valid)
    );

    // 3. Track & Divide: The Centroid Engine
    blob_tracker #(
        .DATA_WIDTH(DATA_WIDTH),
        .MAX_BLOBS(16)
    ) u_blob_tracker (
        .clk(clk),
        .rst_n(rst_n),
        .pixel_i(f_pixel),
        .pixel_x(f_x),
        .pixel_y(f_y),
        .pixel_v(f_valid),
        .frame_done(frame_done),
        .centroid_x(centroid_x),
        .centroid_y(centroid_y),
        .centroid_valid(centroid_valid),
        .centroid_ready(centroid_ready)
    );

endmodule