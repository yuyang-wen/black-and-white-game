module animation (input Clock, enable, resetn, output [2:0]color, output [7:0] x, output [6:0] y);

	wire [2:0]wirec1;
	wire [2:0]wirec2;
	wire [2:0]wirec3;
	wire [2:0]wirec4;
	wire [2:0]wirec5;
	wire [2:0]wirec6;
	wire [2:0]wirec7;
	wire [2:0]frameEnable;
	wire [14:0] wirea;
	wire [7:0] wirex;
	wire [6:0] wirey;
	reg [2:0]wirec;
	reg [2:0] O;

animationDivider a0(
	.Clock(Clock), 
	.resetn(resetn), 
	.Enable(frameEnable));

centerCounter c0(
	.clock(Clock), 
	.resetn(resetn), 
	.enableplotcounter(enable),
	.x(wirex),
	.y(wirey));

always @(posedge Clock) begin
	if (!resetn||!enable)
		O <= 0;

	else if (O == 3'b111) begin
		O <= 0;
	end
	
	else if (frameEnable) begin
		O <= O + 1'b1;
	end		
end

vga_address_translator va0(
	.x(wirex),
	.y(wirey),
	.mem_address(wirea));

blackwhiteclock1 f1(
	.address(wirea),
	.clock(Clock),
	.data(0),
	.wren(0),
	.q(wirec1));
blackwhiteclock2 f2(
	.address(wirea),
	.clock(Clock),
	.data(0),
	.wren(0),
	.q(wirec2));
blackwhiteclock3 f3(
	.address(wirea),
	.clock(Clock),
	.data(0),
	.wren(0),
	.q(wirec3));
blackwhiteclock4 f4(
	.address(wirea),
	.clock(Clock),
	.data(0),
	.wren(0),
	.q(wirec5));
blackwhiteclock5 f5(
	.address(wirea),
	.clock(Clock),
	.data(0),
	.wren(0),
	.q(wirec6));
blackwhiteclock6 f6(
	.address(wirea),
	.clock(Clock),
	.data(0),
	.wren(0),
	.q(wirec7));
blackwhiteclock7 f7(
	.address(wirea),
	.clock(Clock),
	.data(0),
	.wren(0),
	.q(wirec4));

always @ (*) begin
	case (O)
	3'b0 : wirec = wirec1;
	3'b001: wirec = wirec2;
	3'b010: wirec =  wirec3;
	3'b011: wirec =  wirec4;
	3'b100: wirec =  wirec5;
	3'b101: wirec =  wirec6;
	3'b110: wirec = wirec7;
	default: wirec = 3'b111;
	endcase
end

assign x = wirex;
assign y = wirey;
assign color = wirec;

endmodule



module animationDivider(Clock, resetn, Enable);
	input Clock;
	input resetn;
	output reg Enable;

	reg [25:0] Q;

	always @(posedge Clock) begin
		if (!resetn)
			Q <= 0;

		else if (Q == 26'b01011111010111100001000000) begin
			Enable <= 1;
			Q <= 0;
		end
		else begin
			Q <= Q + 26'd1;
			Enable <= 0;
		end
	end
endmodule

module centerCounter(input clock, input resetn, input enableplotcounter, output reg [7:0]x, output reg [6:0]y);
//159 is 10011111
//119 is 1110111
always@(posedge clock) begin

if (!resetn||!enableplotcounter) begin
	x<= 8'd30;
	y<= 7'b0;
end

if (x==8'd131&&enableplotcounter) begin
	x<=8'd30;
	y<=y+7'd1;
end

else if (y==7'd120&&enableplotcounter)begin
	y<=0;
end

else if (enableplotcounter) begin
	x<=x+8'd1;
end

end
endmodule