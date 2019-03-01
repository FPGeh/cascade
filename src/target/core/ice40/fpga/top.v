module top(
    input clk,
    // FIXME: No idea what these inputs do...
    input read,
    input [13:0] vid,
    //input [31:0] in,
    output /*[31:0]*/ out,
    output wait_
);

// UP5K only has 39 I/Os ...
// So drive 'in' with an LFSR
// And XOR all out 'out'
reg [31:0] lfsr;
always @(posedge clk)
    lfsr <= lfsr[31] ^~ lfsr[21] ^~ lfsr[1] ^~ lfsr[0];

wire [31:0] out_vec;
assign out = ^out_vec;

M0 u0 (
  .__clk(clk),
  .__read(read),
  .__vid(vid),
  .__in(in),
  .__out(out_vec),
  .__wait(wait_)
);

endmodule
