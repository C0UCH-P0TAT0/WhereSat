`timescale 1ns / 1ps

module tb_vision_pipeline();

    // 1. System Signals
    reg clk;
    reg rst_n;

    // 2. Camera Stream Interface
    reg [15:0] pixel_in;
    reg        valid_in;
    reg        frame_done;

    // 3. Output Interface
    wire [23:0] centroid_x;
    wire [23:0] centroid_y;
    wire        centroid_valid;
    reg         centroid_ready;

    // 4. Instantiate the DUT
    // We override the threshold to 500 to obliterate the 150-sigma background noise
    vision_pipeline_top #(
        .THRESHOLD(16'd500) 
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .pixel_in(pixel_in),
        .valid_in(valid_in),
        .frame_done(frame_done),
        .centroid_x(centroid_x),
        .centroid_y(centroid_y),
        .centroid_valid(centroid_valid),
        .centroid_ready(centroid_ready)
    );

    // 5. Memory and File Pointers
    reg [15:0] sensor_ram [0:1048575]; // 1024x1024 frame memory
    integer out_file;
    integer i;

    // 6. Hardware Clock Generation (100 MHz)
    initial clk = 0;
    always #5 clk = ~clk;

    // 7. Main Stimulus Engine
    initial begin
        rst_n = 0;
        pixel_in = 0;
        valid_in = 0;
        frame_done = 0;
        centroid_ready = 1; 

        $readmemh("C:/Users/a/Documents/WhereSat/data/tb_frame.mem", sensor_ram);
        out_file = $fopen("C:/Users/a/Documents/WhereSat/fpga/tb/rtl_centroids.txt", "w");

        if (!out_file) begin
            $display("[FATAL ERROR] Could not open output file!");
            $finish;
        end

        #100; rst_n = 1; #20;

        $display("--- 🚀 INITIATING HARDWARE VISION PIPELINE STREAM ---");
        
        // Phase 1: Stream the 1 Megapixel Frame
        for (i = 0; i < 1048576; i = i + 1) begin
            @(posedge clk);
            pixel_in = sensor_ram[i];
            valid_in = 1;
        end

        // Phase 2: Push dummy pixels to force the last rows out of the BRAM delay
        for (i = 0; i < 2050; i = i + 1) begin
            @(posedge clk);
            pixel_in = 0;
            valid_in = 1;
        end

        // Phase 3: Trigger Global Flush
        @(posedge clk);
        frame_done = 1;
        
        // Phase 4: Cut the stream
        @(posedge clk);
        valid_in = 0;
        frame_done = 0;

        // Phase 5: Give the sequential math engine 20,000 ns (2000 cycles) to divide and write
        #20000;

        $display("--- ✅ SIMULATION COMPLETE. CHECK rtl_centroids.txt ---");
        $fclose(out_file);
        $finish;
    end

    // 8. The Q16.8 to Float Extraction Monitor
    always @(posedge clk) begin
        if (centroid_valid && centroid_ready) begin
            $fdisplay(out_file, "%f %f", $itor(centroid_x) / 256.0, $itor(centroid_y) / 256.0);
        end
    end

endmodule