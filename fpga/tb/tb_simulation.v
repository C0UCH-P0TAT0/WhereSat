/**
 * @file tb_simulation.v
 * @brief Standard Verilog Testbench for WhereSat Vision Pipeline.
 * 
 * Features:
 * - 100MHz Clock Generation
 * - 1 Megapixel stream + 2050 dummy pixel buffer
 * - Watchdog timer using standard Verilog while-loop
 * - Q16.8 to Float conversion for subpixel RTL results
 */

`timescale 1ns / 1ps

module tb_simulation();

    // 1. Parameters
    parameter DATA_WIDTH = 16;
    parameter PIXEL_COUNT = 1048576; // 1024 * 1024
    parameter CLK_PERIOD = 10; 
    parameter WATCHDOG_LIMIT = 50000000; // 50ms in ns

    // 2. System Signals
    reg clk;
    reg rst_n;

    // 3. Camera Stream Interface
    reg [DATA_WIDTH-1:0] pixel_in;
    reg                  valid_in;
    reg                  frame_done;

    // 4. Output Interface
    wire [23:0] centroid_x;
    wire [23:0] centroid_y;
    wire [23:0] centroid_m00;
    wire        centroid_valid;
    wire        processing_done;
    reg         centroid_ready;

    // 5. Memory and File Pointers
    reg [DATA_WIDTH-1:0] frame_mem [0:PIXEL_COUNT-1];
    integer file_ptr;
    integer i;
    integer centroid_count;

    // 6. Clock Generation (100 MHz)
    initial begin
        clk = 0;
        forever #(CLK_PERIOD/2) clk = ~clk;
    end

    // 7. Main Stimulus Engine
    initial begin
        // Initialize
        rst_n = 0;
        pixel_in = 0;
        valid_in = 0;
        frame_done = 0;
        centroid_ready = 1; 
        centroid_count = 0;

        // Load Image
        $display("[TB] Loading tb_frame.mem...");
        $readmemh("tb_frame.mem", frame_mem);
        
        // Check if memory loaded (Verilog 'x' check)
        if (frame_mem[0] === {DATA_WIDTH{1'bx}}) begin 
            $display("[FATAL ERROR] tb_frame.mem not found or empty!"); 
            $finish; 
        end

        file_ptr = $fopen("rtl_centroids.txt", "w");
        if (!file_ptr) begin
            $display("[FATAL ERROR] Could not open rtl_centroids.txt!");
            $finish;
        end

        // Reset Sequence
        repeat(10) @(posedge clk);
        rst_n = 1;
        repeat(10) @(posedge clk);

        $display("--- 🚀 STARTING PIXEL STREAM ---");
        
        // Phase 1: Stream the 1 Megapixel Frame
        for (i = 0; i < PIXEL_COUNT; i = i + 1) begin
            @(posedge clk);
            pixel_in = frame_mem[i];
            valid_in = 1;
        end

        // Phase 2: Push 2050 dummy pixels (Clear BRAM/Line Buffer delays)
        $display("[TB] Entering dummy pixel phase...");
        for (i = 0; i < 2050; i = i + 1) begin
            @(posedge clk);
            pixel_in = 0;
            valid_in = 1;
        end

        // Phase 3: Trigger Global Flush
        @(posedge clk);
        frame_done = 1;
        
        // Phase 4: End Stream
        @(posedge clk);
        valid_in = 0;
        frame_done = 0;

        // Phase 5: Watchdog Wait for processing_done
        $display("[TB] Waiting for hardware to quiesce...");
        
        // Standard Verilog while-loop with time-based timeout
        while (processing_done == 0 && $time < WATCHDOG_LIMIT) begin
            @(posedge clk);
        end

        if (processing_done == 0) begin
            $display("[FATAL ERROR] Watchdog Timeout! processing_done never asserted.");
            $finish;
        end

        $display("[TB] processing_done asserted at %0t.", $time);

        // Final margin for file buffer flushing
        repeat(100) @(posedge clk);

        $display("---------------------------------------");
        $display("--- ✅ SIMULATION COMPLETE ---");
        $display("Centroids detected = %0d", centroid_count);
        $display("---------------------------------------");
        
        $fclose(file_ptr);
        $finish;
    end

    // 8. Centroid Monitor (Q16.8 to Float)
    always @(posedge clk) begin
        if (centroid_valid && centroid_ready) begin
            centroid_count = centroid_count + 1;
            // $itor is standard Verilog-2001 for converting integer to real
            $fdisplay(file_ptr, "%f %f %f", 
                     $itor(centroid_x) / 256.0, 
                     $itor(centroid_y) / 256.0, 
                     $itor(centroid_m00));
        end
    end

    // 9. Instantiate the DUT
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
        .centroid_m00(centroid_m00),
        .centroid_valid(centroid_valid),
        .centroid_ready(centroid_ready),
        .processing_done(processing_done)
    );

endmodule