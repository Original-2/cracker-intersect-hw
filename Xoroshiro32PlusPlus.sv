// Generator : SpinalHDL v1.1.5    git head : 0310b2489a097f2b9de5535e02192d9ddd2764ae
// Date      : 02/12/2018, 10:37:58
// Component : Xoroshiro32PlusPlus
// https://github.com/ZiCog/xoroshiro/tree/master

module Xoroshiro32PlusPlus (
      output [31:0] io_prng,
      input         io_next,
      input         clk,
      input         reset
);

  // Internal state
  reg [15:0] s0;
  reg [15:0] s1;
  reg [15:0] p0;
  reg [15:0] p1;

  // Intermediate wires
  wire [15:0] _zz_1;
  wire [15:0] _zz_2;
  wire [15:0] _zz_3;
  wire [15:0] s0_;
  wire [15:0] s1_;
  wire [15:0] random_16;

  // Xoroshiro32++
  assign _zz_3 = ((s1 ^ s0) <<< 5);
  assign s0_   = (({s0[2:0], s0[15:3]} ^ (s1 ^ s0)) ^ _zz_3);
  assign _zz_1 = (s1 ^ s0);
  assign s1_   = {_zz_1[5:0], _zz_1[15:6]};
  assign _zz_2 = (s0 + s1);

  // Original 16‑bit PRNG (p0 + p1, truncated to 16 bits)
  assign random_16 = (p0 + p1);

  // Q16.16 output: integer part = 0, fractional part = random_16 / 65536
  assign io_prng = {16'b0, random_16};

  // State update
  always @(posedge clk or posedge reset) begin
    if (reset) begin
      s0 <= 16'b0000000000000001;
      s1 <= 16'b0000000000000000;
      p0 <= 16'b0000000000000000;
      p1 <= 16'b0000000000000000;
    end else begin
      if (io_next) begin
        s0 <= s0_;
        s1 <= s1_;
        p0 <= {_zz_2[6:0], _zz_2[15:7]};
        p1 <= s0;
      end
    end
  end

endmodule
