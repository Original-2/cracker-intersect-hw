module camera #(
    parameter Q = 16
)(
    input  wire signed [31:0] u,            // Q16.16, 0.0 .. 1.0
    input  wire signed [31:0] v,
    // config
    input  wire signed [31:0] halfWidth,    // Q16.16
    input  wire signed [31:0] halfHeight,
    input  wire signed [31:0] u_vec_x, u_vec_y, u_vec_z,
    input  wire signed [31:0] v_vec_x, v_vec_y, v_vec_z,
    input  wire signed [31:0] w_vec_x, w_vec_y, w_vec_z,
    input  wire signed [31:0] pos_x, pos_y, pos_z,
    // outputs
    output reg  signed [31:0] ro_x, ro_y, ro_z,
    output reg  signed [31:0] rd_x, rd_y, rd_z
);

    // screen_x = (2*u - 1) * halfWidth
    // screen_y = (1 - 2*v) * halfHeight
    wire signed [31:0] two_u      = {u[30:0], 1'b0};            // u << 1, still Q16.16
    wire signed [31:0] two_u_min1 = two_u - 32'sh1_0000;        // 2*u - 1

    wire signed [31:0] two_v      = {v[30:0], 1'b0};
    wire signed [31:0] one_min2v  = 32'sh1_0000 - two_v;        // 1 - 2*v

    wire signed [63:0]  screen_x_m = two_u_min1 * halfWidth;
    wire signed [63:0] screen_y_m = one_min2v  * halfHeight;
    wire signed [31:0] screen_x   = screen_x_m[47:16];          // back to Q16.16
    wire signed [31:0] screen_y   = screen_y_m[47:16];

    // Direction = u_vec * screen_x + v_vec * screen_y - w_vec
    wire signed [63:0] dx1 = u_vec_x * screen_x;
    wire signed [63:0] dx2 = v_vec_x * screen_y;
    wire signed [63:0] dy1 = u_vec_y * screen_x;
    wire signed [63:0] dy2 = v_vec_y * screen_y;
    wire signed [63:0] dz1 = u_vec_z * screen_x;
    wire signed [63:0] dz2 = v_vec_z * screen_y;

    always @* begin
        ro_x = pos_x;
        ro_y = pos_y;
        ro_z = pos_z;

        rd_x = dx1[47:16] + dx2[47:16] - w_vec_x;
        rd_y = dy1[47:16] + dy2[47:16] - w_vec_y;
        rd_z = dz1[47:16] + dz2[47:16] - w_vec_z;
    end

endmodule
