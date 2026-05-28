//   Uses the "fold-back" method

module random_point_on_triangle (
    input  logic signed [31:0] v0_x, v0_y, v0_z,
    input  logic signed [31:0] v1_x, v1_y, v1_z,
    input  logic signed [31:0] v2_x, v2_y, v2_z,
    input  logic signed [31:0] u, v,          // Q16.16, 0 ≤ u,v < 1
    output logic signed [31:0] out_x, out_y, out_z
);

    localparam signed [31:0] ONE   = 32'h0001_0000;
    localparam signed [31:0] ROUND = 32'h0000_8000;  // rounding for mult

    // Internal signals
    logic signed [31:0] u_int, v_int;
    logic signed [31:0] a, b, c;
    logic cond;                         // true when u+v > 1

    // Fold-back logic
    assign cond = (u + v) > ONE;        // compare as unsigned (both positive)
    assign u_int = cond ? (ONE - u) : u;
    assign v_int = cond ? (ONE - v) : v;

    // Barycentric weights (all Q16.16, no negative after correction)
    assign a = ONE - u_int - v_int;     // safe because u_int+v_int <= ONE
    assign b = u_int;
    assign c = v_int;

    // Fixed-point multiplication with rounding
    function automatic signed [31:0] mul_fixed(signed [31:0] x, signed [31:0] y);
        logic signed [63:0] prod;
    begin
        prod = x * y;
        mul_fixed = (prod + ROUND) >>> 16;
    end
    endfunction

    // Weighted sum of vertices
    always_comb begin
        out_x = mul_fixed(a, v0_x) + mul_fixed(b, v1_x) + mul_fixed(c, v2_x);
        out_y = mul_fixed(a, v0_y) + mul_fixed(b, v1_y) + mul_fixed(c, v2_y);
        out_z = mul_fixed(a, v0_z) + mul_fixed(b, v1_z) + mul_fixed(c, v2_z);
    end

endmodule
