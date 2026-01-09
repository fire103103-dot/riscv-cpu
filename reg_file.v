module reg_file (
    input clk,
    input we,
    input [4:0] ra1, ra2, wa,
    input [31:0] wd,
    output [31:0] rd1, rd2
);
    parameter DEPTH = 32;
    
    reg [31:0] mem [0:31];

    // Initialize registers to 0 for simulation
    integer i;
    initial begin
        for (i = 0; i < 32; i = i + 1)
            mem[i] = 32'b0;
    end
    
    assign rd1 = (ra1 == 5'b0) ? 32'b0 : (we && (wa == ra1) && (wa != 5'b0)) ? wd : mem[ra1];
    assign rd2 = (ra2 == 5'b0) ? 32'b0 : (we && (wa == ra2) && (wa != 5'b0)) ? wd : mem[ra2];
    
    always @(posedge clk) begin
        if (we && (wa != 5'b0))
            mem[wa] <= wd;
    end

    // SystemVerilog Assertion: x0 register must always be 0
    always @(posedge clk) begin
        assert (mem[0] == 32'b0) else $error("ASSERTION FAILED: x0 is not zero!");
    end
endmodule
