module pixel_fsm #(
    parameter ADDR_WIDTH = 12
)(
    input  wire                  clk,
    input  wire                  rst_n,

    // C++ hook
    input  wire                  pixel_req,
    output logic                 pixel_ready,
    output logic [31:0]          pixel_r,
    output logic [31:0]          pixel_g,
    output logic [31:0]          pixel_b,

    // Interface to intersection_fsm
    output logic                 isect_start,
    input  wire                  isect_ready,
    input  wire                  isect_hit,
    input  wire [31:0]           isect_hit_idx,

    // Interface to shared BRAM
    output logic                 bram_read_active,
    output logic [ADDR_WIDTH-1:0] bram_addr,
    input  wire  [31:0]          bram_rdata
);

    typedef enum logic [3:0] {
        IDLE       = 4'd0,
        WAIT_ISECT = 4'd1,
        READ_AX    = 4'd2,
        READ_AY    = 4'd3,
        READ_AZ    = 4'd4,
        READ_EX    = 4'd5,
        READ_EY    = 4'd6,
        READ_EZ    = 4'd7,
        FINISH_EY  = 4'd8,
        FINISH_EZ  = 4'd9
    } state_t;

    state_t state;
    logic [31:0] base_addr;
    logic [31:0] ax, ay, az;

    always @(posedge clk) begin
        if (!rst_n) begin
            state <= IDLE;
            pixel_ready <= 0;
            isect_start <= 0;
            bram_read_active <= 0;
            bram_addr <= 0;
            pixel_r <= 0; pixel_g <= 0; pixel_b <= 0;
        end else begin
            case (state)
                IDLE: begin
                    pixel_ready <= 0;
                    if (pixel_req) begin
                        isect_start <= 1;
                        state <= WAIT_ISECT;
                    end
                end

                WAIT_ISECT: begin
                    isect_start <= 0; // Turn off the request pulse

                    // Wait for the intersector to finish
                    if (isect_ready && !isect_start) begin
                        if (isect_hit) begin
                            base_addr <= isect_hit_idx * 16;
                            bram_read_active <= 1;
                            state <= READ_AX;
                        end else begin
                            // Miss: Return black background
                            pixel_r <= 0; pixel_g <= 0; pixel_b <= 0;
                            pixel_ready <= 1;
                            state <= IDLE;
                        end
                    end
                end

                // Pipelined BRAM Reads
                // We request address N+1 while latching the data for address N
                READ_AX: begin
                    bram_addr <= base_addr + 9;  // Req Albedo X
                    state <= READ_AY;
                end
                READ_AY: begin
                    bram_addr <= base_addr + 10; // Req Albedo Y
                    state <= READ_AZ;
                end
                READ_AZ: begin
                    bram_addr <= base_addr + 11; // Req Albedo Z
                    ax <= bram_rdata;            // Latch Albedo X
                    state <= READ_EX;
                end
                READ_EX: begin
                    bram_addr <= base_addr + 12; // Req Emission X
                    ay <= bram_rdata;            // Latch Albedo Y
                    state <= READ_EY;
                end
                READ_EY: begin
                    bram_addr <= base_addr + 13; // Req Emission Y
                    az <= bram_rdata;            // Latch Albedo Z
                    state <= READ_EZ;
                end
                READ_EZ: begin
                    bram_addr <= base_addr + 14; // Req Emission Z
                    pixel_r <= ax + bram_rdata;  // r = AlbedoX + EmissionX
                    state <= FINISH_EY;
                end
                FINISH_EY: begin
                    pixel_g <= ay + bram_rdata;  // g = AlbedoY + EmissionY
                    state <= FINISH_EZ;
                end
                FINISH_EZ: begin
                    pixel_b <= az + bram_rdata;  // b = AlbedoZ + EmissionZ
                    bram_read_active <= 0;
                    pixel_ready <= 1;
                    state <= IDLE;
                end

                default: state <= IDLE;
            endcase
        end
    end
endmodule
