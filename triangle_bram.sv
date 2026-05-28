module triangle_bram #(
    parameter MAX_TRIANGLES = 256,
    parameter WORDS_PER_TRI = 16,   // 9 vertices + 3 albedo + 3 emission + 1 flags
    parameter ADDR_WIDTH = $clog2(MAX_TRIANGLES * WORDS_PER_TRI)
)(
    input  logic                clk,
    input  logic [ADDR_WIDTH-1:0] cpu_addr,
    input  logic [31:0]         cpu_data,
    input  logic                cpu_we,
    input  logic [ADDR_WIDTH-1:0] fsm_addr,
    output logic [31:0]         fsm_data
);
    (* ram_style = "block" *) logic [31:0] mem [0:MAX_TRIANGLES*WORDS_PER_TRI-1];

    always_ff @(posedge clk) begin
        if (cpu_we) mem[cpu_addr] <= cpu_data;
        fsm_data <= mem[fsm_addr];
    end
endmodule
