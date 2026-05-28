module top #(
    parameter MAX_TRIANGLES = 256,
    parameter ADDR_WIDTH = $clog2(MAX_TRIANGLES * 16)
)(
    input wire                   clk,
    input wire                   rst_n,

    // Intersection FSM Ports
    input wire                   scene_valid,
    input wire [31:0]            scene_ignore_idx,
    input wire [31:0]            ray_origin_x, ray_origin_y, ray_origin_z,
    input wire [31:0]            ray_dir_x, ray_dir_y, ray_dir_z,
    output logic                 scene_ready,
    output logic                 scene_hit,
    output logic [31:0]          scene_t,
    output logic [31:0]          scene_hit_idx,

    // New Pixel FSM Ports
    input wire                   pixel_req,
    output logic                 pixel_ready,
    output logic [31:0]          pixel_r,
    output logic [31:0]          pixel_g,
    output logic [31:0]          pixel_b,

    // RNG ports
    output wire [31:0]           io_prng,
    input wire                   io_next,

    // Random point on triangle
    input wire [31:0]            rpt_v0_x, rpt_v0_y, rpt_v0_z,
    input wire [31:0]            rpt_v1_x, rpt_v1_y, rpt_v1_z,
    input wire [31:0]            rpt_v2_x, rpt_v2_y, rpt_v2_z,
    input wire [31:0]            rpt_u, rpt_v,
    output wire [31:0]           rpt_out_x, rpt_out_y, rpt_out_z,

    // Camera ports
    input wire [31:0]            cam_u, cam_v,
    input wire [31:0]            cam_halfWidth, cam_halfHeight,
    input wire [31:0]            cam_u_vec_x, cam_u_vec_y, cam_u_vec_z,
    input wire [31:0]            cam_v_vec_x, cam_v_vec_y, cam_v_vec_z,
    input wire [31:0]            cam_w_vec_x, cam_w_vec_y, cam_w_vec_z,
    input wire [31:0]            cam_pos_x, cam_pos_y, cam_pos_z,
    output wire [31:0]           cam_ro_x, cam_ro_y, cam_ro_z,
    output wire [31:0]           cam_rd_x, cam_rd_y, cam_rd_z,

    // BRAM ports (CPU Muxed)
    input logic [ADDR_WIDTH-1:0] bram_addr,
    input logic [31:0]           bram_wdata,
    input logic                  bram_we,
    input logic [ADDR_WIDTH-1:0] cpu_bram_addr,
    output logic [31:0]          cpu_bram_rdata
);

    // Arbitration & Mode Tracking
    logic running_pixel_fsm;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            running_pixel_fsm <= 1'b0;
        end else begin
            if (pixel_req) begin
                running_pixel_fsm <= 1'b1;
            end else if (pixel_ready) begin
                running_pixel_fsm <= 1'b0;
            end
        end
    end

    // BRAM Multiplexing
    // Priority: 1. Intersection Hardware 2. Pixel Shader Pipeline 3. CPU Read/Write

    logic [ADDR_WIDTH-1:0] fsm_bram_addr;
    logic [31:0]           fsm_bram_rdata;
    logic                  fsm_bram_active;

    logic [ADDR_WIDTH-1:0] pixel_bram_addr;
    logic                  pixel_bram_active;

    logic [ADDR_WIDTH-1:0] muxed_bram_addr;

   assign muxed_bram_addr = fsm_bram_active   ? fsm_bram_addr :
                             pixel_bram_active ? pixel_bram_addr :
                             cpu_bram_addr;

    triangle_bram #(.MAX_TRIANGLES(MAX_TRIANGLES)) bram (
        .clk      (clk),
        .cpu_addr (bram_addr),
        .cpu_data (bram_wdata),
        .cpu_we   (bram_we),
        .fsm_addr (muxed_bram_addr),
        .fsm_data (fsm_bram_rdata)
    );

    assign cpu_bram_rdata = fsm_bram_rdata;

    // Ray Multiplexing for Intersector Core
    logic [31:0] int_ray_origin_x, int_ray_origin_y, int_ray_origin_z;
    logic [31:0] int_ray_dir_x, int_ray_dir_y, int_ray_dir_z;

    // Route math vectors depending on who initiated the trace execution
    assign int_ray_origin_x = running_pixel_fsm ? cam_ro_x : ray_origin_x;
    assign int_ray_origin_y = running_pixel_fsm ? cam_ro_y : ray_origin_y;
    assign int_ray_origin_z = running_pixel_fsm ? cam_ro_z : ray_origin_z;

    assign int_ray_dir_x    = running_pixel_fsm ? cam_rd_x : ray_dir_x;
    assign int_ray_dir_y    = running_pixel_fsm ? cam_rd_y : ray_dir_y;
    assign int_ray_dir_z    = running_pixel_fsm ? cam_rd_z : ray_dir_z;

    // Intersector Core Unit
    logic isect_valid, isect_ready, isect_hit;
    logic [31:0] isect_t;
    logic [31:0] v0_x, v0_y, v0_z;
    logic [31:0] v1_x, v1_y, v1_z;
    logic [31:0] v2_x, v2_y, v2_z;

    ray_triangle_intersect intersector (
        .clk(clk),
        .rst_n(rst_n),
        .valid(isect_valid),
        .ray_origin_x(int_ray_origin_x), .ray_origin_y(int_ray_origin_y), .ray_origin_z(int_ray_origin_z),
        .ray_dir_x(int_ray_dir_x),       .ray_dir_y(int_ray_dir_y),       .ray_dir_z(int_ray_dir_z),
        .v0_x(v0_x), .v0_y(v0_y), .v0_z(v0_z),
        .v1_x(v1_x), .v1_y(v1_y), .v1_z(v1_z),
        .v2_x(v2_x), .v2_y(v2_y), .v2_z(v2_z),
        .ready(isect_ready),
        .hit(isect_hit),
        .t(isect_t)
    );

    // Hardware Intersection State Machine
    logic internal_scene_valid;

    intersection_fsm #(
        .ADDR_WIDTH(ADDR_WIDTH)
    ) fsm_inst (
        .clk(clk),
        .rst_n(rst_n),

        // Logical OR allows either explicit C++ port signals OR the internal Pixel FSM to initiate traces
        .scene_valid(scene_valid | internal_scene_valid),
        .scene_ignore_idx(scene_ignore_idx),
        .scene_ready(scene_ready),
        .scene_hit(scene_hit),
        .scene_t(scene_t),
        .scene_hit_idx(scene_hit_idx),

        // BRAM Fetching Interface
        .bram_rdata(fsm_bram_rdata),
        .bram_addr(fsm_bram_addr),
        .bram_read_active(fsm_bram_active),

        // Core Intersector Interface
        .isect_ready(isect_ready),
        .isect_hit(isect_hit),
        .isect_t(isect_t),
        .isect_valid(isect_valid),
        .v0_x(v0_x), .v0_y(v0_y), .v0_z(v0_z),
        .v1_x(v1_x), .v1_y(v1_y), .v1_z(v1_z),
        .v2_x(v2_x), .v2_y(v2_y), .v2_z(v2_z)
    );

    // Pixel Pipeline FSM
    pixel_fsm #(
        .ADDR_WIDTH(ADDR_WIDTH)
    ) pixel_fsm_inst (
        .clk(clk),
        .rst_n(rst_n),

        // C++ Hook
        .pixel_req(pixel_req),
        .pixel_ready(pixel_ready),
        .pixel_r(pixel_r),
        .pixel_g(pixel_g),
        .pixel_b(pixel_b),

        // Loopback signals to Scene Intersector FSM
        .isect_start(internal_scene_valid),
        .isect_ready(scene_ready),
        .isect_hit(scene_hit),
        .isect_hit_idx(scene_hit_idx),

        // Secondary BRAM Reader control
        .bram_read_active(pixel_bram_active),
        .bram_addr(pixel_bram_addr),
        .bram_rdata(fsm_bram_rdata)
    );

    // Math Helpers & Execution Blocks
    Xoroshiro32PlusPlus rng (
        .io_prng(io_prng), .io_next(io_next), .clk(clk), .reset(~rst_n)
    );

    random_point_on_triangle random_point (
        .v0_x(rpt_v0_x), .v0_y(rpt_v0_y), .v0_z(rpt_v0_z),
        .v1_x(rpt_v1_x), .v1_y(rpt_v1_y), .v1_z(rpt_v1_z),
        .v2_x(rpt_v2_x), .v2_y(rpt_v2_y), .v2_z(rpt_v2_z),
        .u(rpt_u), .v(rpt_v),
        .out_x(rpt_out_x), .out_y(rpt_out_y), .out_z(rpt_out_z)
    );

    camera cam_inst (
        .u(cam_u), .v(cam_v), .halfWidth(cam_halfWidth), .halfHeight(cam_halfHeight),
        .u_vec_x(cam_u_vec_x), .u_vec_y(cam_u_vec_y), .u_vec_z(cam_u_vec_z),
        .v_vec_x(cam_v_vec_x), .v_vec_y(cam_v_vec_y), .v_vec_z(cam_v_vec_z),
        .w_vec_x(cam_w_vec_x), .w_vec_y(cam_w_vec_y), .w_vec_z(cam_w_vec_z),
        .pos_x(cam_pos_x), .pos_y(cam_pos_y), .pos_z(cam_pos_z),
        .ro_x(cam_ro_x), .ro_y(cam_ro_y), .ro_z(cam_ro_z),
        .rd_x(cam_rd_x), .rd_y(cam_rd_y), .rd_z(cam_rd_z)
    );

endmodule
