`timescale 1ns / 1ps

module blob_tracker #(
    parameter DATA_WIDTH = 16,
    parameter MAX_BLOBS = 16,
    parameter MIN_BLOB_MASS = 24'd0,    // Gate opened for MCU-side filtering
    parameter MIN_HITS = 5'd2,
    parameter MAX_STAR_WIDTH = 10'd12
)(
    input  wire                   clk,
    input  wire                   rst_n,
    input  wire [DATA_WIDTH-1:0]  pixel_i,
    input  wire [9:0]             pixel_x,
    input  wire [9:0]             pixel_y,
    input  wire                   pixel_v,
    input  wire                   frame_done,
    output reg [23:0]             centroid_x,    
    output reg [23:0]             centroid_y,    
    output reg [23:0]             centroid_m00,  // Added: Mass output
    output reg                    centroid_valid,
    input  wire                   centroid_ready,
    output wire                   processing_done
);

    // --- Slot Memory ---
    reg [MAX_BLOBS-1:0] slot_active;
    reg [9:0]           slot_x_min [0:MAX_BLOBS-1], slot_x_max [0:MAX_BLOBS-1], slot_last_y [0:MAX_BLOBS-1];
    reg [23:0]          slot_m00 [0:MAX_BLOBS-1];
    reg [39:0]          slot_m10 [0:MAX_BLOBS-1], slot_m01 [0:MAX_BLOBS-1];
    reg [4:0]           slot_px_cnt [0:MAX_BLOBS-1]; 

    reg [3:0]  flush_ptr;
    reg        global_flush_mode, frame_seen;
    reg [1:0]  state;
    localparam S_IDLE = 0, S_DIVIDE = 1, S_WAIT_READY = 2;

    assign processing_done = frame_seen && !global_flush_mode && (slot_active == 16'h0) && (state == S_IDLE);

    // --- Matching Logic ---
    reg [MAX_BLOBS-1:0] match_bus;
    integer i;
    always @(*) begin
        for (i = 0; i < MAX_BLOBS; i = i + 1) begin
            match_bus[i] = slot_active[i] && 
                           (pixel_x >= (slot_x_min[i] > 10'd2 ? slot_x_min[i] - 10'd2 : 10'd0)) && 
                           (pixel_x <= (slot_x_max[i] < 10'd1021 ? slot_x_max[i] + 10'd2 : 10'd1023)) && 
                           (pixel_x <= (slot_x_min[i] + MAX_STAR_WIDTH)) && 
                           (pixel_x >= (slot_x_max[i] > MAX_STAR_WIDTH ? slot_x_max[i] - MAX_STAR_WIDTH : 10'd0)) && 
                           (pixel_y <= ((slot_last_y[i] >= 10'd1021) ? 10'd1023 : slot_last_y[i] + 10'd2)); 
        end
    end

    wire [3:0] match_idx = match_bus[0]?0:match_bus[1]?1:match_bus[2]?2:match_bus[3]?3:match_bus[4]?4:match_bus[5]?5:match_bus[6]?6:match_bus[7]?7:match_bus[8]?8:match_bus[9]?9:match_bus[10]?10:match_bus[11]?11:match_bus[12]?12:match_bus[13]?13:match_bus[14]?14:15;
    wire [3:0] empty_idx = !slot_active[0]?0:!slot_active[1]?1:!slot_active[2]?2:!slot_active[3]?3:!slot_active[4]?4:!slot_active[5]?5:!slot_active[6]?6:!slot_active[7]?7:!slot_active[8]?8:!slot_active[9]?9:!slot_active[10]?10:!slot_active[11]?11:!slot_active[12]?12:!slot_active[13]?13:!slot_active[14]?14:15;

    reg [47:0] d_accum; reg [23:0] d_divisor; reg [5:0] d_cnt; reg [39:0] d_quot; reg d_mode_y;

    integer fd, j;
    initial fd = $fopen("slot_debug.txt", "w");

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            slot_active <= 0; flush_ptr <= 0; global_flush_mode <= 0; frame_seen <= 0;
            state <= S_IDLE; centroid_valid <= 0; centroid_x <= 0; centroid_y <= 0; centroid_m00 <= 0;
            for (i = 0; i < MAX_BLOBS; i = i + 1) begin
                slot_m00[i] <= 0; slot_m10[i] <= 0; slot_m01[i] <= 0;
                slot_px_cnt[i] <= 0; slot_x_min[i] <= 0; slot_x_max[i] <= 0; slot_last_y[i] <= 0;
            end
        end else begin
            if (frame_done) begin 
                $fdisplay(fd, "\nTIME: %0t | FRAME DONE", $time);
                for(j = 0; j < MAX_BLOBS; j = j + 1) begin
                    $fdisplay(fd, "Slot %2d: Act=%b Mass=%10d Width=%2d", j, slot_active[j], slot_m00[j], 
                        (slot_x_max[j] >= slot_x_min[j]) ? (slot_x_max[j]-slot_x_min[j]) : 0);
                end
                global_flush_mode <= 1'b1; frame_seen <= 1'b1; 
            end

            if (pixel_v && pixel_i != 0 && !global_flush_mode) begin
                if (|match_bus) begin
                    slot_m00[match_idx] <= slot_m00[match_idx] + pixel_i;
                    slot_m10[match_idx] <= slot_m10[match_idx] + (pixel_i * pixel_x);
                    slot_m01[match_idx] <= slot_m01[match_idx] + (pixel_i * pixel_y);
                    if (slot_px_cnt[match_idx] != 5'd31) slot_px_cnt[match_idx] <= slot_px_cnt[match_idx] + 5'd1;
                    if (pixel_y > slot_last_y[match_idx]) slot_last_y[match_idx] <= pixel_y;
                    if (pixel_x < slot_x_min[match_idx]) slot_x_min[match_idx] <= pixel_x;
                    if (pixel_x > slot_x_max[match_idx]) slot_x_max[match_idx] <= pixel_x;
                end else if (|(~slot_active)) begin
                    slot_active[empty_idx] <= 1'b1; slot_m00[empty_idx] <= pixel_i;
                    slot_m10[empty_idx] <= (pixel_i * pixel_x); slot_m01[empty_idx] <= (pixel_i * pixel_y);
                    slot_px_cnt[empty_idx] <= 5'd1; slot_x_min[empty_idx] <= pixel_x;
                    slot_x_max[empty_idx] <= pixel_x; slot_last_y[empty_idx] <= pixel_y;
                end
            end

            case (state)
                S_IDLE: begin
                    if (slot_active[flush_ptr] && (global_flush_mode || (pixel_y > ((slot_last_y[flush_ptr] >= 10'd1021) ? 10'd1023 : slot_last_y[flush_ptr] + 10'd2)))) begin
                        if (slot_m00[flush_ptr] >= MIN_BLOB_MASS && slot_px_cnt[flush_ptr] >= MIN_HITS) begin 
                            state <= S_DIVIDE; d_cnt <= 0; d_mode_y <= 0; d_quot <= 0;
                            d_accum <= {slot_m10[flush_ptr], 8'b0}; d_divisor <= slot_m00[flush_ptr];
                        end else begin
                            slot_active[flush_ptr] <= 0; slot_m00[flush_ptr] <= 0;
                            if (flush_ptr == MAX_BLOBS-1) flush_ptr <= 0; else flush_ptr <= flush_ptr + 1;
                            if (flush_ptr == MAX_BLOBS-1 && global_flush_mode) global_flush_mode <= 0;
                        end
                    end else begin
                        if (flush_ptr == MAX_BLOBS-1) flush_ptr <= 0; else flush_ptr <= flush_ptr + 1;
                        if (flush_ptr == MAX_BLOBS-1 && global_flush_mode) global_flush_mode <= 0;
                    end
                end
                S_DIVIDE: begin
                    if (d_cnt < 40) begin
                        if (d_accum >= ({24'b0, d_divisor} << (39-d_cnt))) begin
                            d_accum <= d_accum - ({24'b0, d_divisor} << (39-d_cnt));
                            d_quot[39-d_cnt] <= 1'b1;
                        end else d_quot[39-d_cnt] <= 1'b0;
                        d_cnt <= d_cnt + 1;
                    end else begin
                        if (!d_mode_y) begin
                            centroid_x <= d_quot[23:0]; d_mode_y <= 1; d_cnt <= 0; d_quot <= 0;
                            d_accum <= {slot_m01[flush_ptr], 8'b0}; d_divisor <= slot_m00[flush_ptr];
                        end else begin
                            centroid_y <= d_quot[23:0]; 
                            centroid_m00 <= slot_m00[flush_ptr]; // Output the mass
                            centroid_valid <= 1; state <= S_WAIT_READY;
                        end
                    end
                end
                S_WAIT_READY: if (centroid_valid && centroid_ready) begin
                    centroid_valid <= 0; slot_active[flush_ptr] <= 0; slot_m00[flush_ptr] <= 0;
                    if (flush_ptr == MAX_BLOBS-1) flush_ptr <= 0; else flush_ptr <= flush_ptr + 1;
                    if (flush_ptr == MAX_BLOBS-1 && global_flush_mode) global_flush_mode <= 0;
                    state <= S_IDLE;
                end
            endcase
        end
    end
endmodule