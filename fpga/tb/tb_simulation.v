`timescale 1ns / 1ps

module tb_simulation();

    parameter DATA_WIDTH = 16, IMAGE_WIDTH = 1024, PIXEL_COUNT = 1024*1024, CLK_PERIOD = 10; 

    reg clk, rst_n, valid_in, frame_done, centroid_ready;
    reg [DATA_WIDTH-1:0] pixel_in;
    wire [23:0] centroid_x, centroid_y;
    wire centroid_valid, processing_done;

    reg [DATA_WIDTH-1:0] frame_mem [0:PIXEL_COUNT-1];
    integer file_ptr, i;
    integer centroid_count = 0; // Added: Counter for summary

    initial begin clk = 0; forever #(CLK_PERIOD/2) clk = ~clk; end

    initial begin
        file_ptr = $fopen("rtl_centroids.txt", "w");
        if (file_ptr == 0) begin $display("ERROR: Could not open rtl_centroids.txt"); $finish; end

        $display("Loading tb_frame.mem...");
        $readmemh("tb_frame.mem", frame_mem);
        if (frame_mem[0] === {DATA_WIDTH{1'bx}}) begin $display("ERROR: tb_frame.mem not loaded."); $finish; end

        rst_n = 0; pixel_in = 0; valid_in = 0; frame_done = 0; centroid_ready = 1;
        repeat(10) @(posedge clk); rst_n = 1; repeat(10) @(posedge clk);

        $display("Streaming pixels...");
        for (i = 0; i < PIXEL_COUNT; i = i + 1) begin
            pixel_in = frame_mem[i];
            valid_in = 1;
            @(posedge clk);
        end

        valid_in = 0; pixel_in = 0; frame_done = 1;
        @(posedge clk);
        frame_done = 0;

        $display("Waiting for processing_done...");
        wait(processing_done);
        $display("processing_done asserted.");

        repeat(10) @(posedge clk);
        $fclose(file_ptr);
        $display("---------------------------------------");
        $display("Simulation Finished.");
        $display("Centroids detected = %0d", centroid_count); // Added: Summary report
        $display("---------------------------------------");
        $finish;
    end

    // Collection block with counter
    always @(posedge clk) begin
        if (centroid_valid && centroid_ready) begin
            centroid_count = centroid_count + 1;
            $fwrite(file_ptr, "%d %d\n", centroid_x, centroid_y);
        end
    end

    vision_pipeline_top #(.THRESHOLD(16'd100)) dut (
        .clk(clk), .rst_n(rst_n), .pixel_in(pixel_in), .valid_in(valid_in), .frame_done(frame_done),
        .centroid_x(centroid_x), .centroid_y(centroid_y), .centroid_valid(centroid_valid),
        .centroid_ready(centroid_ready), .processing_done(processing_done)
    );

endmodule