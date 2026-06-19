`timescale 1ns / 1ps

// WhereSat Project: FPGA Vision Pipeline
// Module: blob_tracker (Production Version)
// Description: 16-slot Bounding Box Tracker. Defensive architecture with Ingest Gate,
// Strict saturated bounds, State Purging, Saturated Footprint Counter, and Parameterized Thresholds.

module blob_tracker #(
    parameter DATA_WIDTH = 16,
    parameter MAX_BLOBS = 16,
    parameter MIN_PIXEL_VAL = 16'd500, // The Ingest Gate (Intensity)
    parameter MIN_MASS = 24'd5000,     // The Mass Gate (Total Flux)
    parameter MIN_HITS = 5'd3          // The Footprint Gate (Pixels > Threshold)
)(
    input  wire                   clk,
    input  wire                   rst_n,
    
    // From threshold_filter
    input  wire [DATA_WIDTH-1:0]  pixel_i,
    input  wire [9:0]             pixel_x,
    input  wire [9:0]             pixel_y,
    input  wire                   pixel_v,
    input  wire                   frame_done,
    
    // To MCU Interface (AXI-Stream Style)
    output reg [23:0]             centroid_x,    // Q16.8
    output reg [23:0]             centroid_y,    // Q16.8
    output reg                    centroid_valid,
    input  wire                   centroid_ready
);

    // --- Slot Memory ---
    reg [MAX_BLOBS-1:0] slot_active;
    reg [9:0]           slot_x_min [0:MAX_BLOBS-1];
    reg [9:0]           slot_x_max [0:MAX_BLOBS-1];
    reg [9:0]           slot_last_y [0:MAX_BLOBS-1];
    reg [23:0]          slot_m00   [0:MAX_BLOBS-1];
    reg [39:0]          slot_m10   [0:MAX_BLOBS-1];
    reg [39:0]          slot_m01   [0:MAX_BLOBS-1];
    reg [4:0]           slot_px_cnt[0:MAX_BLOBS-1]; // Footprint Hit Counter

    // --- Matching Logic ---
    reg [MAX_BLOBS-1:0] match_bus;
    integer i;
    always @(*) begin
        for (i = 0; i < MAX_BLOBS; i = i + 1) begin
            match_bus[i] = slot_active[i] && 
                           (pixel_x >= slot_x_min[i]) && (pixel_x <= slot_x_max[i]) && 
                           (pixel_y >= slot_last_y[i]) &&       
                           (pixel_y <= ((slot_last_y[i] >= 10'd1021) ? 10'd1023 : slot_last_y[i] + 10'd2)); 
        end
    end

    wire [3:0] match_idx, empty_idx;
    wire       match_found, empty_found;
    assign match_found = |match_bus;

    assign match_idx = match_bus[0]  ? 0  : match_bus[1]  ? 1  : match_bus[2]  ? 2  : match_bus[3]  ? 3  :
                       match_bus[4]  ? 4  : match_bus[5]  ? 5  : match_bus[6]  ? 6  : match_bus[7]  ? 7  :
                       match_bus[8]  ? 8  : match_bus[9]  ? 9  : match_bus[10] ? 10 : match_bus[11] ? 11 :
                       match_bus[12] ? 12 : match_bus[13] ? 13 : match_bus[14] ? 14 : 15;

    assign empty_found = |(~slot_active);
    assign empty_idx   = !slot_active[0]  ? 0  : !slot_active[1]  ? 1  : !slot_active[2]  ? 2  : !slot_active[3]  ? 3  :
                         !slot_active[4]  ? 4  : !slot_active[5]  ? 5  : !slot_active[6]  ? 6  : !slot_active[7]  ? 7  :
                         !slot_active[8]  ? 8  : !slot_active[9]  ? 9  : !slot_active[10] ? 10 : !slot_active[11] ? 11 :
                         !slot_active[12] ? 12 : !slot_active[13] ? 13 : !slot_active[14] ? 14 : 15;

    // --- Control Logic ---
    reg [3:0]  flush_ptr;
    reg        global_flush_mode;
    reg [1:0]  state;
    localparam S_IDLE = 0, S_DIVIDE = 1, S_WAIT_READY = 2;

    reg [47:0] d_accum;
    reg [23:0] d_divisor;
    reg [5:0]  d_cnt;
    reg [39:0] d_quot;
    reg        d_mode_y;

    wire [9:0] calc_x_min = (pixel_x <= 10'd2)    ? 10'd0    : pixel_x - 10'd2;
    wire [9:0] calc_x_max = (pixel_x >= 10'd1021) ? 10'd1023 : pixel_x + 10'd2;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            slot_active <= 0; flush_ptr <= 0; global_flush_mode <= 0;
            state <= S_IDLE; centroid_valid <= 0;
        end else begin
            if (frame_done) global_flush_mode <= 1'b1;

            if (pixel_v && pixel_i >= MIN_PIXEL_VAL && !global_flush_mode) begin
                if (match_found) begin
                    slot_m00[match_idx] <= slot_m00[match_idx] + pixel_i;
                    slot_m10[match_idx] <= slot_m10[match_idx] + (pixel_i * pixel_x);
                    slot_m01[match_idx] <= slot_m01[match_idx] + (pixel_i * pixel_y);
                    
                    // [PATCH] Saturated Footprint Counter
                    if (slot_px_cnt[match_idx] != 5'd31) begin
                        slot_px_cnt[match_idx] <= slot_px_cnt[match_idx] + 5'd1;
                    end
                    
                    if (calc_x_min < slot_x_min[match_idx]) slot_x_min[match_idx] <= calc_x_min;
                    if (calc_x_max > slot_x_max[match_idx]) slot_x_max[match_idx] <= calc_x_max;
                    
                    if (pixel_y > slot_last_y[match_idx]) slot_last_y[match_idx] <= pixel_y;
                    
                end else if (empty_found) begin
                    slot_active[empty_idx]  <= 1'b1;
                    slot_m00[empty_idx]     <= pixel_i;
                    slot_m10[empty_idx]     <= (pixel_i * pixel_x);
                    slot_m01[empty_idx]     <= (pixel_i * pixel_y);
                    slot_px_cnt[empty_idx]  <= 5'd1; 
                    
                    slot_x_min[empty_idx]   <= calc_x_min;
                    slot_x_max[empty_idx]   <= calc_x_max;
                    slot_last_y[empty_idx]  <= pixel_y;
                end
            end

            case (state)
                S_IDLE: begin
                    if (slot_active[flush_ptr] && (global_flush_mode || (pixel_y > ((slot_last_y[flush_ptr] >= 10'd1021) ? 10'd1023 : slot_last_y[flush_ptr] + 10'd2)))) begin
                        
                        // [PATCH] Parameterized Footprint Gate
                        if (slot_m00[flush_ptr] >= MIN_MASS && slot_px_cnt[flush_ptr] >= MIN_HITS) begin 
                            state       <= S_DIVIDE; 
                            d_cnt       <= 0; 
                            d_mode_y    <= 0;
                            d_accum     <= {slot_m10[flush_ptr], 8'b0};
                            
                            d_divisor   <= slot_m00[flush_ptr];
                            d_quot      <= 40'd0; 
                        end else begin
                            slot_active[flush_ptr] <= 1'b0;
                            slot_m00[flush_ptr]    <= 0;
                            slot_m10[flush_ptr]    <= 0;
                            slot_m01[flush_ptr]    <= 0;
                            slot_x_min[flush_ptr]  <= 0;
                            slot_x_max[flush_ptr]  <= 0;
                            slot_last_y[flush_ptr] <= 0;
                            slot_px_cnt[flush_ptr] <= 0;
                            
                            flush_ptr <= flush_ptr + 1;
                            if (flush_ptr == 15 && global_flush_mode) global_flush_mode <= 0;
                        end
                    end else begin
                        flush_ptr <= flush_ptr + 1;
                        if (flush_ptr == 15 && global_flush_mode) global_flush_mode <= 0;
                    end
                end

                S_DIVIDE: begin
                    if (d_cnt < 40) begin
                        if (d_accum >= ({24'b0, d_divisor} << (39-d_cnt))) begin
                            d_accum <= d_accum - ({24'b0, d_divisor} << (39-d_cnt));
                            d_quot[39-d_cnt] <= 1;
                        end else d_quot[39-d_cnt] <= 0;
                        d_cnt <= d_cnt + 1;
                    end else begin
                        if (!d_mode_y) begin
                            centroid_x <= d_quot[23:0];
                            d_mode_y   <= 1'b1; 
                            d_cnt      <= 0;
                            d_accum    <= {slot_m01[flush_ptr], 8'b0};
                            d_divisor  <= slot_m00[flush_ptr]; 
                            d_quot     <= 40'd0; 
                        end else begin
                            centroid_y <= d_quot[23:0];
                            centroid_valid <= 1'b1;
                            state <= S_WAIT_READY;
                        end
                    end
                end

                S_WAIT_READY: begin
                    if (centroid_valid && centroid_ready) begin
                        centroid_valid <= 1'b0;
                        slot_active[flush_ptr] <= 1'b0;
                        
                        slot_m00[flush_ptr]    <= 0;
                        slot_m10[flush_ptr]    <= 0;
                        slot_m01[flush_ptr]    <= 0;
                        slot_x_min[flush_ptr]  <= 0;
                        slot_x_max[flush_ptr]  <= 0;
                        slot_last_y[flush_ptr] <= 0;
                        slot_px_cnt[flush_ptr] <= 0;
                        
                        flush_ptr <= flush_ptr + 1;
                        if (flush_ptr == 15 && global_flush_mode) global_flush_mode <= 0;
                        
                        state <= S_IDLE;
                    end
                end
            endcase
        end
    end
endmodule