module datapathVGA(
	input clk, 
	input resetn,
	input enableplotcounter,
	input [5:0]colourmux,
	output [7:0]x, 
	output [6:0]y, 
	output reg [2:0] colour
	);
	
	wire [2:0] wireplayer1;
	wire [2:0] wireplayer2;
	wire [2:0] wirestart;

	always @(*)begin //mux for picking which colour we are outputing
		case (colourmux)
		6'd0: colour = wirestart;
		6'd1: colour = wireplayer1;
		6'd2: colour = wireplayer2;
		
		6'd3: colour = wireplayedblack;
		6'd4: colour = wireplayedwhite;
		6'd5: colour = wireplayer1round;
		6'd6: colour = wireplayer2round;
		6'd7: colour = wireplayer1won;
		6'd8: colour = wireplayer2won;

		6'd9: colour = wireplayedblack;
		6'd10: colour = wireplayedwhite;

		6'd11: colour = scoreboard;

		6'd12: colour = tie;

		default: colour = 3'd0; //random color 
		endcase
	end
	wire [7:0] wirex;
	wire [6:0] wirey;
	screenCounter s1(
		.clock(clk),
		.resetn(resetn),
		.enableplotcounter(enableplotcounter), //enable plot counter signal comes from the fsm
		.x(wirex), //sends the x and y coordinates
		.y(wirey) //sends the x and y coordinates
		);

	assign x = wirex;
	assign y = wirey;

	wire [14:0]wireadress;

	vga_address_translator v0(
	.x(wirex),
	.y(wirey),
	.mem_address(wireadress));
	
	initialbackground i1(
	.address(wireadress),
	.clock(clk),
	.data(0),
	.wren(0),
	.q(wirestart));
	
	player1turn p0(
	.address(wireadress),
	.clock(clk),
	.data(0),
	.wren(0),
	.q(wireplayer1));

	player2turn p1(
	.address(wireadress),
	.clock(clk),
	.data(0),
	.wren(0),
	.q(wireplayer2));

	wire [2:0] wireplayedblack;
	wire [2:0] wireplayedwhite;
	wire [2:0] wireplayer1round;
	wire [2:0] wireplayer2round;
	wire [2:0] wireplayer1won;
	wire [2:0] wireplayer2won;
	wire [2:0] tie;

	playedblack p2(
	.address(wireadress),
	.clock(clk),
	.data(0),
	.wren(0),
	.q(wireplayedblack));
	
	playedwhite p3(
	.address(wireadress),
	.clock(clk),
	.data(0),
	.wren(0),
	.q(wireplayedwhite));
	
	player1round p4(
	.address(wireadress),
	.clock(clk),
	.data(0),
	.wren(0),
	.q(wireplayer1round));

	player2round p5(
	.address(wireadress),
	.clock(clk),
	.data(0),
	.wren(0),
	.q(wireplayer2round));

	player1won p6(
	.address(wireadress),
	.clock(clk),
	.data(0),
	.wren(0),
	.q(wireplayer1won));

	player2won p7(
	.address(wireadress),
	.clock(clk),
	.data(0),
	.wren(0),
	.q(wireplayer2won));	
	
	wire [2:0] scoreboard;

	score_board p8(
	.address(wireadress),
	.clock(clk),
	.data(0),
	.wren(0),
	.q(scoreboard));
	
	tie p9(
	.address(wireadress),
	.clock(clk),
	.data(0),
	.wren(0),
	.q(tie));

endmodule