module top
	(
		CLOCK_50,						//	On Board 50 MHz
		// Your inputs and outputs here
		KEY,
		SW,
		LEDR,
		HEX0,
		HEX1,
		HEX2,
		HEX3,

		// The ports below are for the VGA output.  Do not change.
		VGA_CLK,   						//	VGA Clock
		VGA_HS,							//	VGA H_SYNC
		VGA_VS,							//	VGA V_SYNC
		VGA_BLANK_N,						//	VGA BLANK
		VGA_SYNC_N,						//	VGA SYNC
		VGA_R,   						//	VGA Red[9:0]
		VGA_G,	 						//	VGA Green[9:0]
		VGA_B,   						//	VGA Blue[9:0]


        //audio 
        AUD_ADCDAT,

        // Bidirectionals
        AUD_BCLK,
        AUD_ADCLRCK,
        AUD_DACLRCK,

        FPGA_I2C_SDAT,

        // Outputs
        AUD_XCK,
        AUD_DACDAT,

        FPGA_I2C_SCLK

	);

	input			CLOCK_50;				//	50 MHz
	// Declare your inputs and outputs here
	input [3:0] KEY;
	input [9:0] SW;
	output [9:0] LEDR;
	output [6:0] HEX1;
	output [6:0] HEX2;
	output [6:0] HEX3;
	output [6:0] HEX0;
	// Do not change the following outputs
	output			VGA_CLK;   				//	VGA Clock
	output			VGA_HS;					//	VGA H_SYNC
	output			VGA_VS;					//	VGA V_SYNC
	output			VGA_BLANK_N;				//	VGA BLANK
	output			VGA_SYNC_N;				//	VGA SYNC
	output	[9:0]	VGA_R;   				//	VGA Red[9:0]
	output	[9:0]	VGA_G;	 				//	VGA Green[9:0]
	output	[9:0]	VGA_B;   				//	VGA Blue[9:0]
	
    //audio
    input               AUD_ADCDAT;

    // Bidirectionals
    inout               AUD_BCLK;
    inout               AUD_ADCLRCK;
    inout               AUD_DACLRCK;

    inout               FPGA_I2C_SDAT;

    // Outputs
    output              AUD_XCK;
    output              AUD_DACDAT;

    output              FPGA_I2C_SCLK;


    Internal Wires
    wire                audio_in_available;
    wire        [31:0]  left_channel_audio_in;
    wire        [31:0]  right_channel_audio_in;
    wire                read_audio_in;

    wire                audio_out_allowed;
    wire        [31:0]  left_channel_audio_out;
    wire        [31:0]  right_channel_audio_out;
    wire                write_audio_out;

	// wire resetn;
	// assign resetn = KEY[3];

 //    reg signed [31:0] audiostart;
 //    reg signed [31:0] csharp;

 //    always @(*) begin
 //        if (1)begin
 //            audiostart = {audioamplitude, zero };
 //        end

 //        else begin
 //            audiostart = 32'b0;
 //        end

 //        if (e_csharp || (done == 0)) begin
 //            csharp =  {audioamplitudecsharp, zero };
 //        end

 //        else begin
 //            csharp = 32'b0;
 //        end

 //    end 

    wire [31:0] sound;

 //    wire signed [5:0] audioamplitude;
 //    wire signed [5:0] audioamplitudecsharp;
 //    wire signed [25:0] zero;
 //    wire done;
 //    // wire signed [25:0] zerocsharp;
 //    assign zero = 26'b0;
 //    // assign zerocsharp = 
 //    wire Enable;
 //    wire Enablecsharp;
 //    wire [19:0] addressaudio;
 //    wire [15:0] addressaudiocsharp;

 //    wire e_csharp;
 //    wire resetcsharp;

 //    RateDivideraudio r10(CLOCK_50, KEY[3], Enable, 1); 
 //    countto377974 c0(CLOCK_50, KEY[3], Enable, addressaudio); 

 //    always @(posedge CLOCK_50) begin
 //        if (done == 1) begin
 //            // reset
 //            Enablecounter44100hz <= 0;
 //        end
 //        else if (done == 0) begin
 //            Enablecounter44100hz <= e_csharp;
 //        end
 //    end

 //    reg Enablecounter44100hz;

 //    RateDivideraudio r11(CLOCK_50, resetn, Enablecsharp, Enablecounter44100hz); 
 //    countto48279 c2(CLOCK_50, ~resetcsharp, Enablecsharp, addressaudiocsharp, done); 

 //    audiostartRAM s2(0,audioamplitude,addressaudio,0,CLOCK_50);
 //    csharp c1(0,audioamplitudecsharp,addressaudiocsharp,0,CLOCK_50);

    assign read_audio_in            = audio_in_available & audio_out_allowed;

    assign left_channel_audio_out   = left_channel_audio_in+sound;
    assign right_channel_audio_out  = right_channel_audio_in+sound;
    assign write_audio_out          = audio_in_available & audio_out_allowed;

    /*****************************************************************************
     *                              Internal Modules                             *
     *****************************************************************************/

    Audio_Controller Audio_Controller (
        // Inputs
        .CLOCK_50                       (CLOCK_50),
        .reset                      (~KEY[3]),

        .clear_audio_in_memory      (),
        .read_audio_in              (read_audio_in),
        
        .clear_audio_out_memory     (),
        .left_channel_audio_out     (left_channel_audio_out),
        .right_channel_audio_out    (right_channel_audio_out),
        .write_audio_out            (write_audio_out),

        .AUD_ADCDAT                 (AUD_ADCDAT),

        // Bidirectionals
        .AUD_BCLK                   (AUD_BCLK),
        .AUD_ADCLRCK                (AUD_ADCLRCK),
        .AUD_DACLRCK                (AUD_DACLRCK),


        // Outputs
        .audio_in_available         (audio_in_available),
        .left_channel_audio_in      (left_channel_audio_in),
        .right_channel_audio_in     (right_channel_audio_in),

        .audio_out_allowed          (audio_out_allowed),

        .AUD_XCK                    (AUD_XCK),
        .AUD_DACDAT                 (AUD_DACDAT)

    );

    avconf #(.USE_MIC_INPUT(1)) avc (
        .FPGA_I2C_SCLK                  (FPGA_I2C_SCLK),
        .FPGA_I2C_SDAT                  (FPGA_I2C_SDAT),
        .CLOCK_50                   (CLOCK_50),
        .reset                      (~KEY[3])
    );

	
	wire writeEn;
	// assign writeEn = ~KEY[1];
	
	// Create the colour, x, y and writeEn wires that are inputs to the controller.

	wire [2:0] colour;
	wire [7:0] x;
	wire [6:0] y;
	// Create an Instance of a VGA controller - there can be only one!
	// Define the number of colours as well as the initial background
	// image file (.MIF) for the controller.
	vga_adapter VGA(
			.resetn(resetn),
			.clock(CLOCK_50),
			.colour(colour),
			.x(x),
			.y(y),
			.plot(writeEn), //writeEn
			/* Signals for the DAC to drive the monitor. */
			.VGA_R(VGA_R),
			.VGA_G(VGA_G),
			.VGA_B(VGA_B),
			.VGA_HS(VGA_HS),
			.VGA_VS(VGA_VS),
			.VGA_BLANK(VGA_BLANK_N),
			.VGA_SYNC(VGA_SYNC_N),
			.VGA_CLK(VGA_CLK));
		defparam VGA.RESOLUTION = "160x120";
		defparam VGA.MONOCHROME = "TRUE";
		defparam VGA.BITS_PER_COLOUR_CHANNEL = 1;
		defparam VGA.BACKGROUND_IMAGE = "initialbackground.mif";
			
	// Put your code here. Your code should produce signals x,y,colour and writeEn
	// for the VGA controller, in addition to any other functionality your design may require.


	blackandwhite bw0(
		.CLOCK_50(CLOCK_50),
		.KEY(KEY),
		.SW(SW), 
		.LEDR(LEDR),
		.HEX1(HEX1), 
		.HEX2(HEX2), 
		.HEX3(HEX3), 
		.HEX0(HEX0), 
		.enableplot(writeEn), 
		.x(x), 
		.y(y), 
		.colour(colour),
        .sound(sound)
		);


	
endmodule

module screenCounter (input clock, input resetn, input enableplotcounter, output reg [7:0]x, output reg [6:0]y);
//159 is 10011111
//119 is 1110111
always@(posedge clock) begin

if (!resetn||!enableplotcounter) begin
	y<= 0;
	x<= 0;
end

if (x==8'd160&&enableplotcounter) begin
	x<=0;
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

module blackandwhite (CLOCK_50, KEY, SW, LEDR, HEX1, HEX2, HEX3, HEX0, enableplot, x, y, colour, sound);
input CLOCK_50;
input [3:0] KEY;
input [9:0] SW; //reset is SW[0]
output [9:0] LEDR;
output [6:0] HEX1;
output [6:0] HEX2;
output [6:0] HEX3;
output [6:0] HEX0;
output enableplot;
output reg [7:0] x;
output reg [6:0] y;
output reg [2:0] colour;
output [31:0] sound;

wire [2:0] count1;
wire [2:0] count2;
wire [1:0] playeronewin;
wire e_load1;
wire e_load2;
wire e_compare;

wire Enable;
wire counter30;
wire counter3;
wire Enablecounter30;
wire Enablecounter3;
wire Disablecounter30;
wire e_reset;
wire enableplotcounter;
wire enableplot;
wire [5:0] colourmux;
wire p1black;
wire p2black;
wire enableAnimations;
wire EnableScreen1;
wire screen1;

wire [2:0]wirec;
wire [7:0]wirex;
wire [6:0]wirey;

wire [2:0]animationc;
wire [7:0]animationx;
wire [6:0]animationy;

wire p1subtract;
wire p2subtract;
wire e_hex1;
wire e_hex2;

wire e_csharp ;
wire resetcsharp ;

control u0(
    .clk(CLOCK_50),
    .resetn(KEY[3]),
    .go(~KEY[0]),
    .counter3(counter3),
    .counter30(counter30),
    .screen1(screen1),
    .enableScreen1(EnableScreen1),
    .count1(count1),
    .count2(count2),
    .playeronewin(playeronewin),
    .p1black(p1black),
    .p2black(p2black),
    .e_load1(e_load1),
    .e_load2(e_load2),
    .e_hex1(e_hex1),
    .e_hex2(e_hex2),
    .e_animation(enableAnimations),
    .e_compare(e_compare),
    .LEDR(LEDR),
    .Enablecounter3(Enablecounter3),
    .Enablecounter30(Enablecounter30),
    .Disablecounter30(Disablecounter30),
    .e_reset(e_reset),
    .enableplotcounter(enableplotcounter),
    .enableplot(enableplot),
    .colourmux(colourmux),
    .enableline(enableline),
    .p2subtract(p2subtract),
    .p1subtract(p1subtract),
    .e_csharp (e_csharp), 
    .resetcsharp(resetcsharp) 
    );
datapathVGA d1(
	.clk(CLOCK_50), 
	.resetn(KEY[3]),
	.enableplotcounter(enableplotcounter),
	.colourmux(colourmux),
	.x(wirex), 
	.y(wirey), 
	.colour(wirec)
	);

animation a10 (
    .Clock(CLOCK_50),
    .enable(enableAnimations), 
    .resetn(KEY[3]),
    .color(animationc), 
    .x(animationx), 
    .y(animationy));

datapathAudio a11(
    .CLOCK_50(CLOCK_50),
    .resetn(KEY[3]),
    .e_csharp(),
    .resetcsharp(),
    .sound(sound)
    );

datapath u1(
    .clk(CLOCK_50),
    .resetn(KEY[3]),
    .data_in(datain),
    .e_load1(e_load1),
    .e_load2(e_load2),
    .e_compare(e_compare),
    .e_reset (e_reset),
    .playeronewin(playeronewin),
    .count1(count1),
    .count2(count2),
    .p1black(p1black),
    .p2black(p2black),

	.HEX2(HEX2),
	.HEX3(HEX3),

    .p1state(p1state),
    .p2state(p2state),
    .p1subtract(p1subtract),
    .p2subtract(p2subtract),
    .e_hex1(e_hex1),
    .e_hex2(e_hex2)
    );

reg [5:0]datain;

inputProcess i1(
    .in1(KEY[1]), 
    .in10(KEY[2]), 
    .resetn(KEY[3]), 
    .out1(out11), 
    .out10(out101),
    .data(datain1),
    .enable(e_hex1));

inputProcess i2(
    .in1(KEY[1]), 
    .in10(KEY[2]), 
    .resetn(KEY[3]), 
    .out1(out12), 
    .out10(out102),
    .data(datain2),
    .enable(e_hex2));

reg [3:0]out1;
reg [3:0]out10;
wire [3:0]out11;
wire [3:0]out101;
wire [5:0]datain1;
wire [3:0]out12;
wire [3:0]out102;
wire [5:0]datain2;

always @ (*) begin
    if (e_hex1) begin
        datain = datain1;
        out1 = out11;
        out10 = out101;
    end
    else if (e_hex2) begin
        datain = datain2;
        out1 = out12;
        out10 = out102;
    end	 
    else begin
        out1 = 4'b0;
        out10 = 4'b0;
    end

end

hex_decoder hd1(
    .hex_digit(out10),
    .segments(HEX1));

hex_decoder hd2(
    .hex_digit(out1),
    .segments(HEX0));


wire [2:0] linec;
wire [7:0] linex;
wire [6:0] liney;

wire [2:0] p1state;
wire [2:0] p2state;

wire enableline;

drawScore ds0 (CLOCK_50, KEY[3], enableline, p1state , p2state , linec, linex, liney);

RateDivider u5(CLOCK_50, KEY[3], counter3, Enablecounter3); //for 3 seconds

Screen1 sc1(CLOCK_50, KEY[3], screen1, EnableScreen1);

always @ (*)begin
  if (enableAnimations==1'b1)begin
        x=animationx;
        y=animationy;
        colour=animationc;
   end
   else if (enableline==1'b1)begin
        x=linex;
        y=liney;
        colour=linec;
   end
   else begin
        x=wirex;
        y=wirey;
        colour=wirec;
   end
end

endmodule

module control(
    input clk,
    input resetn,
    input go,
    input counter3,
    input counter30,
    input [2:0] count1,
    input [2:0] count2,
    input [1:0] playeronewin,
    input screen1,
    input p1black,
    input p2black,
    output reg e_load1,
    output reg e_load2,
	 output reg e_hex1,
	 output reg e_hex2,
    output reg e_compare,
    output reg e_animation,
    output reg [9:0] LEDR,
    output reg Enablecounter3,
    output reg Enablecounter30,
    output reg Disablecounter30,
    output reg e_reset,
    output reg enableplotcounter,
    output reg enableplot,
    output reg [5:0]colourmux,
    output reg enableScreen1,
    output reg enableline,
    output reg p1subtract,
    output reg p2subtract, 
    output reg e_csharp, 
    output reg resetcsharp
    );

    reg [5:0] current_state, next_state; 
    // reg count;
    
    localparam  
                S_RESET         = 6'd0,
                S_RESET_WAIT    = 6'd1,

                S_P1_FIRST      = 6'd2,
                S_P1_FIRST_WAIT = 6'd3,

                S_P2_LOAD       = 6'd4,
                S_P2_LOAD_WAIT  = 6'd5,

                S_COMPARE       = 6'd6,

                S_WINNER        = 6'd7,
                S_WINNER_WAIT_1 = 6'd8,
                S_WINNER_WAIT_2 = 6'd9,

                S_P2_FIRST      = 6'd10,
                S_P2_FIRST_WAIT = 6'd11,

                S_P1_LOAD       = 6'd12,
                S_P1_LOAD_WAIT  = 6'd13,

                S_GAME_END1     = 6'd14,
                S_GAME_END2      = 6'd15,

                S_P1_FIRST_BLACK = 6'd16,
                S_P2_LOAD_BLACK = 6'd17,
                S_P2_FIRST_BLACK = 6'd18,
                S_P1_LOAD_BLACK = 6'd19,

                S_TIMER_1 = 6'd20,
                S_TIMER_2 = 6'd21,
                S_TIMER_3 = 6'd22,
                S_TIMER_4 = 6'd23,

                S_DRAW_BOARD_1 = 6'd24,
                S_DRAW_BOARD_2 = 6'd25,
                S_DRAW_BOARD_3 = 6'd26,
                S_DRAW_BOARD_4 = 6'd27,

                S_DRAW_SCORE_1 = 6'd28,
                S_DRAW_SCORE_2 = 6'd29,
                S_DRAW_SCORE_3 = 6'd30,
                S_DRAW_SCORE_4 = 6'd31;
    
    // Next state logic aka our state table
    always@(*)
    begin: state_table 
        case (current_state)
            S_RESET: next_state = go ? S_RESET_WAIT : S_RESET;
            S_RESET_WAIT: next_state = go ? S_RESET_WAIT: S_P1_FIRST;

            S_P1_FIRST: next_state =  counter3 ? S_DRAW_BOARD_1 : S_P1_FIRST;
            //score
            S_DRAW_BOARD_1: next_state = screen1 ? S_DRAW_SCORE_1 : S_DRAW_BOARD_1 ;
            S_DRAW_SCORE_1: next_state = screen1 ? S_TIMER_1 : S_DRAW_SCORE_1 ;
            //timer animation
            S_TIMER_1: next_state = go ? S_P1_FIRST_WAIT : S_TIMER_1;
            S_P1_FIRST_WAIT: next_state = go ? S_P1_FIRST_WAIT: S_P1_FIRST_BLACK;
            S_P1_FIRST_BLACK: next_state = counter3 ? S_P2_LOAD: S_P1_FIRST_BLACK ;

            S_P2_LOAD: next_state = counter3 ? S_DRAW_BOARD_2: S_P2_LOAD;
            //score
            S_DRAW_BOARD_2: next_state = screen1 ? S_DRAW_SCORE_2 : S_DRAW_BOARD_2 ;
            S_DRAW_SCORE_2: next_state = screen1 ? S_TIMER_2 : S_DRAW_SCORE_2 ;
            //timer animation
            S_TIMER_2: next_state = go ? S_P2_LOAD_WAIT : S_TIMER_2;
            S_P2_LOAD_WAIT: next_state = go ? S_P2_LOAD_WAIT :S_P2_LOAD_BLACK;
            S_P2_LOAD_BLACK: next_state = counter3 ? S_COMPARE : S_P2_LOAD_BLACK;

            S_COMPARE: next_state = S_WINNER;

            S_WINNER: next_state = playeronewin[0] ? S_WINNER_WAIT_1: S_WINNER_WAIT_2;
            S_WINNER_WAIT_1: next_state = counter3 ? S_P1_FIRST :S_WINNER_WAIT_1 ;
            S_WINNER_WAIT_2: next_state = counter3 ? S_P2_FIRST :S_WINNER_WAIT_2 ;

            S_P2_FIRST: next_state = counter3 ? S_DRAW_BOARD_3: S_P2_FIRST;
            //score
            S_DRAW_BOARD_3: next_state = screen1 ? S_DRAW_SCORE_3 : S_DRAW_BOARD_3 ;
            S_DRAW_SCORE_3: next_state = screen1 ? S_TIMER_3 : S_DRAW_SCORE_3 ;
            //timer animation
            S_TIMER_3: next_state = go ? S_P2_FIRST_WAIT : S_TIMER_3;
            S_P2_FIRST_WAIT: next_state = go ? S_P2_FIRST_WAIT :S_P2_FIRST_BLACK;
            S_P2_FIRST_BLACK: next_state = counter3 ? S_P1_LOAD : S_P2_FIRST_BLACK; 

            S_P1_LOAD: next_state = counter3 ? S_DRAW_BOARD_4: S_P1_LOAD;
            //score
            S_DRAW_BOARD_4: next_state = screen1 ? S_DRAW_SCORE_4 : S_DRAW_BOARD_4 ;
            S_DRAW_SCORE_4: next_state = screen1 ? S_TIMER_4 : S_DRAW_SCORE_4 ;
            //timer animation
            S_TIMER_4: next_state = go ? S_P1_LOAD_WAIT : S_TIMER_4;
            S_P1_LOAD_WAIT: next_state = go ? S_P1_LOAD_WAIT :S_P1_LOAD_BLACK ;
            S_P1_LOAD_BLACK: next_state = counter3 ? S_COMPARE :  S_P1_LOAD_BLACK;

            S_GAME_END1: next_state = counter3 ? S_RESET : S_GAME_END1;
            S_GAME_END2: next_state = counter3 ? S_RESET : S_GAME_END2;

            default: next_state =  S_RESET;
        endcase
    end // state_table
   

    // Output logic aka all of our datapath control signals
    always @(*)
    begin: enable_signals
        // By default make all our signals 0
        e_load1 = 1'b0;
        e_load2 = 1'b0;
        e_compare = 1'b0;
        LEDR[9:0] = 10'b0;
        Enablecounter30 = 0;
        Enablecounter3 = 0; 
        Disablecounter30 = 0;
        e_reset = 0;
        enableplotcounter = 1'b1;
        enableplot =1'b0;
        colourmux = 5'd0;//default at start screen
        e_animation = 1'b0;
        enableline = 1'b0;
		  enableScreen1 = 0;
          p1subtract = 0;
          p2subtract = 0;
			 e_hex1 = 0;
			 e_hex2 = 0;
        e_csharp = 0;
        resetcsharp = 0;



        case (current_state)
            S_RESET: begin
                e_reset = 1;
                colourmux = 6'd0;
                enableplot =1'b1;
                enableplotcounter = 1'b1;
					 
                resetcsharp = 1;


            end
            S_P1_FIRST: begin 
                
                 LEDR[0] = 1;
                 colourmux = 6'd1;
                 enableplot =1'b1;
                 enableplotcounter = 1'b1;
                 Enablecounter3 = 1'b1;
					  resetcsharp = 1; 

            end
            S_P1_FIRST_WAIT: begin
                 e_load1 = 1'b1;
                 colourmux = 6'd1;
            end
            S_P1_FIRST_BLACK: begin
                Enablecounter3 =1;

                if (p1black == 1'b1)begin
                    colourmux = 6'd9;
                end
                else begin
                    colourmux = 6'd10;
                end
                enableplot =1'b1;
                enableplotcounter = 1'b1;
                p1subtract = 1'b1;

            end
            S_P2_LOAD: begin 
                 LEDR[1] = 1;
                 colourmux = 6'd2;
                  Enablecounter3 = 1'b1;
                  enableplot =1'b1;
                  enableplotcounter = 1'b1;
						
            end
            S_P2_LOAD_WAIT: begin
	            	 e_load2 = 1'b1;
	            	 colourmux = 6'd2;
	                 Disablecounter30 = 1;
                     end
            S_P2_LOAD_BLACK: begin
                Enablecounter3 =1;

                if (p2black == 1'b1)begin
                    colourmux = 6'd9;
                end
                else begin
                    colourmux = 6'd10;
                end
                enableplot =1'b1;
                enableplotcounter = 1'b1;
                p2subtract = 1'b1;
            end


            S_P2_FIRST: begin
                     LEDR[2] = 1;
                     colourmux = 6'd2;
                      Enablecounter3 = 1'b1;
                      enableplot =1'b1;
                      enableplotcounter = 1'b1;
					 resetcsharp = 1;

            end
            S_P2_FIRST_WAIT: begin 
            	e_load2 = 1'b1;
            	colourmux = 6'd2;
                Disablecounter30 = 1;
                     end

            S_P2_FIRST_BLACK: begin
                Enablecounter3 =1;

                if (p2black == 1'b1)begin
                    colourmux = 6'd9;
                end
                else begin
                    colourmux = 6'd10;
                end
                enableplot =1'b1;
                enableplotcounter = 1'b1;
                p2subtract = 1'b1;
            end
            S_P1_LOAD: begin
                 LEDR[3] = 1;
                 colourmux = 6'd1;
                  Enablecounter3 = 1'b1;
                  enableplot =1'b1;
                  enableplotcounter = 1'b1;

            end
            S_P1_LOAD_WAIT: begin 
        		e_load1 = 1'b1;
        		colourmux = 6'd1;
                 Disablecounter30 = 1;
                     end
            S_P1_LOAD_BLACK: begin
                Enablecounter3 =1;

                if (p1black == 1'b1)begin
                    colourmux = 6'd9;
                end
                else begin
                    colourmux = 6'd10;
                end
                enableplot =1'b1;
                enableplotcounter = 1'b1;
                p1subtract = 1'b1;

            end

            S_COMPARE: begin 
                e_compare = 1'b1;
                        LEDR[4] = 1;
 
            end
            S_WINNER_WAIT_1: begin
                LEDR[9] = 1;
                Enablecounter3 =1;

                colourmux = 6'd5;
                enableplot =1'b1;
                enableplotcounter = 1'b1;
					 
                e_csharp = 1;					 


            end
            S_WINNER_WAIT_2: begin
                LEDR[8] = 1;
                Enablecounter3 = 1;
                if (playeronewin[1]==0)
                colourmux = 6'd6;
                if (playeronewin[1]==1)
                colourmux = 6'd12;
                enableplot =1'b1;
                enableplotcounter = 1'b1;
                e_csharp = 1;					 
 

            end
            S_GAME_END1: begin
                LEDR[7] =1;
                Enablecounter3 = 1;
                e_reset =1;

                colourmux = 6'd7;
                enableplot =1'b1;
                enableplotcounter = 1'b1;

                e_csharp = 1;

            end 
            S_GAME_END2: begin
                LEDR[6]=1;
                Enablecounter3 =1;
                e_reset=1;

                colourmux = 6'd8;
                enableplot =1'b1;
                enableplotcounter = 1'b1;


                e_csharp = 1;

                end
            S_TIMER_1: begin 
					 e_animation = 1'b1;
					 enableplot = 1'b1;
                     e_hex1 = 1'b1;
					 end
            S_TIMER_2: begin 
					 e_animation = 1'b1;
					 enableplot = 1'b1;
                     e_hex2 = 1'b1; 
					 end
            S_TIMER_3: begin 
					 e_animation = 1'b1;
					 enableplot = 1'b1;
                     e_hex2 = 1'b1; 
					 end
            S_TIMER_4: begin 
					 e_animation = 1'b1; 
					 enableplot = 1'b1;
                     e_hex1 = 1'b1; 
					 end
            S_DRAW_BOARD_1: begin
                    enableScreen1 = 1'b1;
                    enableplot = 1'b1;
                    colourmux = 6'd11;    
                    end

            S_DRAW_BOARD_2: begin
                    enableScreen1 = 1'b1;
                    enableplot = 1'b1;
                    colourmux = 6'd11;    
                    end

            S_DRAW_BOARD_3: begin
                    enableScreen1 = 1'b1;
                    enableplot = 1'b1;
                    colourmux = 6'd11;    
                    end

            S_DRAW_BOARD_4: begin
                    enableScreen1 = 1'b1;
                    enableplot = 1'b1;
                    colourmux = 6'd11;    
                    end
            S_DRAW_SCORE_1: begin
                    enableScreen1 = 1'b1;
                    enableplot = 1'b1; 
                    enableline = 1'b1;   
                    end

            S_DRAW_SCORE_2: begin
                    enableScreen1 = 1'b1;
                    enableplot = 1'b1; 
                    enableline = 1'b1;   
                    end

            S_DRAW_SCORE_3: begin
                    enableScreen1 = 1'b1;
                    enableplot = 1'b1;
                    enableline = 1'b1;
                    end

            S_DRAW_SCORE_4: begin
                    enableScreen1 = 1'b1;
                    enableplot = 1'b1;  
                    enableline = 1'b1;
                    end

                     
        // default:    // don't need default since we already made sure all of our outputs were assigned a value at the start of the always block
        endcase
    end // enable_signals
    // current_state rekgisters
    always@(posedge clk ) //or posedge black
    begin: state_FFs
        if(!resetn) begin
            current_state <= S_RESET;
        end
        else if(count1==3'b101)
            current_state <= S_GAME_END1;
        else if(count2==3'b101)
            current_state <= S_GAME_END2;
        else
            current_state <= next_state;
    end // state_FFS

endmodule

module datapath(
    input clk,
    input resetn,
    input [5:0] data_in,
    input e_load1, e_load2,
    input e_compare,
    input e_reset,
    input p2subtract,
    input p1subtract,
    input e_hex1,
    input e_hex2,
    output reg [1:0] playeronewin,
    output reg [2:0] count1,
    output reg [2:0] count2,
    output reg p1black,
    output reg p2black,
	 output [6:0] HEX2,
	 output [6:0] HEX3,
	 output reg [2:0] p1state,
    output reg [2:0] p2state	 
    );
    
    // input registers
    reg [5:0] p1, p2, x1, x2;
	 reg [5:0] hexdisplay;

	 always @ (*) begin
		if (e_hex1) hexdisplay = p1;
		else if (e_hex2) hexdisplay = p2;
		else hexdisplay = 6'b0;
	 end
    wire [3:0] hexdisplay1;
    wire [3:0] hexdisplay10;
    assign hexdisplay1 = hexdisplay % 4'd10;
    assign hexdisplay10 = hexdisplay / 4'd10;
		hex_decoder h1(
						.hex_digit(hexdisplay1),
						.segments(HEX2));
		hex_decoder h2(
						.hex_digit(hexdisplay10),
						.segments(HEX3));
						
    always @(*) begin
        if (p1 >= 6'd40)
        p1state = 3'd0;
        else if (p1 >= 6'd30 && p1 < 6'd40)
        p1state = 3'd1;
        else if (p1 >= 6'd20 && p1 < 6'd30)
        p1state = 3'd2;
        else if (p1 >= 6'd10 && p1 < 6'd20)
        p1state = 3'd3;
        else 
        p1state = 3'd4;
    end

    always @(*) begin
        if (p2 >= 6'd40)
        p2state = 3'd0;
        else if (p2 >= 6'd30 && p2 < 6'd40)
        p2state = 3'd1;
        else if (p2 >= 6'd20 && p2 < 6'd30)
        p2state = 3'd2;
        else if (p2 >= 6'd10 && p2 < 6'd20)
        p2state = 3'd3;
        else 
        p2state = 3'd4;
    end
    // Registers a, b, c, x with respective input logic
    always@(posedge clk) begin
        if(!resetn || e_reset) begin
 //starts at 49
            x1 <= 6'b0; 
            x2 <= 6'b0; 
        end
        else begin
            if (e_load1)begin
					 if (x1<p1)
                x1 <= data_in;
					 else begin
					 x1 <= p1;
					 end
            end
            else if (e_load2)begin
					 if (x2<p2)
                x2 <= data_in;
					
					 else begin
					 x2 <= p2;
					 end
            end
				
        end
    end
    //determining the output
      always @(posedge clk) begin
		if (!resetn || e_reset)begin  
            p1 <= 6'b 110001; //starts at 49 
            p2 <= 6'b 110001;
        count1 <= 3'b0;
        count2 <= 3'b0; 
    end
	 else if (p1subtract)
		  p1 <= p1 - x1;
        else if (p2subtract)
		   p2 <= p2 - x2;
    else if (e_compare)
        begin

        
		  
        if (x1>x2) begin
            playeronewin <= 2'b01; //player one wins
            count1 <= count1 + 3'b001;
        end
        else if (x1<x2) begin
            playeronewin <= 2'b00; //player two wins
            count2 <= count2 + 3'b001;
        end
        else if (x1==x2) begin
            playeronewin <= 2'b10; //tie 
            count1 <= count1 +3'b001;
            count2 <= count2 +3'b001;
        end

    end
	 end

    always @(*) begin

        //for the black and white
        if (x1 < 6'd5)begin
            p1black = 1'b1;
        end
        else if (x1 >= 6'd5) begin
            p1black = 1'b0;
		  end
		  else p1black = 1'b0;
        
        if (x2 < 6'd5)begin
            p2black = 1'b1;
        end
        else if (x2 >= 6'd5) begin
            p2black = 1'b0;
        end
		  else p2black = 1'b0;
		  
    end
endmodule

module hex_decoder(hex_digit, segments);
    input [3:0] hex_digit;
    output reg [6:0] segments;
   
    always @(*)
        case (hex_digit)
            4'h0: segments = 7'b100_0000;
            4'h1: segments = 7'b111_1001;
            4'h2: segments = 7'b010_0100;
            4'h3: segments = 7'b011_0000;
            4'h4: segments = 7'b001_1001;
            4'h5: segments = 7'b001_0010;
            4'h6: segments = 7'b000_0010;
            4'h7: segments = 7'b111_1000;
            4'h8: segments = 7'b000_0000;
            4'h9: segments = 7'b001_1000;
            4'hA: segments = 7'b000_1000;
            4'hB: segments = 7'b000_0011;
            4'hC: segments = 7'b100_0110;
            4'hD: segments = 7'b010_0001;
            4'hE: segments = 7'b000_0110;
            4'hF: segments = 7'b000_1110;   
            default: segments = 7'h7f;
        endcase
endmodule

module RateDivider(Clock, resetn, Enable, Enablecounter3); //for 3 seconds
    input Clock;
    input resetn;
    output reg Enable;
    input Enablecounter3;

    reg [27:0] Q;

    always @(posedge Clock) begin
        if (!resetn)
            Q <= 0;

        else if (Q == 28'b1000111100001101000110000000) begin //50 000 000 times 3
            Enable <= 1;
            Q <= 0;
        end

        else if (Enablecounter3) begin
            Q <= Q + 28'd1;
            Enable <= 0;
        end
    end
endmodule

module Screen1(Clock, resetn, Enable, EnableScreen1); //for 3 seconds
    input Clock;
    input resetn;
    output reg Enable;
    input EnableScreen1;

    reg [16:0] Q;

    always @(posedge Clock) begin
        if (!resetn)
            Q <= 0;

        else if (Q == 15'b100111000100000) begin //50 000 000 times 3
            Enable <= 1;
            Q <= 0;
        end

        else if (EnableScreen1) begin
            Q <= Q + 15'd1;
            Enable <= 0;
        end
    end
endmodule

module inputProcess (input in1, input in10, input resetn, output [3:0]out1, output [3:0] out10, output [5:0] data, input enable);

reg [3:0]hexOut1;
reg [3:0]hexOut10;

always@(negedge in1) begin
    if(!resetn)begin
        hexOut1<=4'b0;
    end
    else if (hexOut1==4'b1001)begin
        hexOut1<=4'b0;
    end
    else if (enable) begin
        hexOut1<=hexOut1+1'b1;
    end
end

always@(negedge in10) begin
    if(!resetn)begin
        hexOut10<=4'b0;
    end
    else if (hexOut10==4'b0100)begin
        hexOut10<=4'b0;
    end
    else if (enable) begin
        hexOut10<=hexOut10+1'b1;
    end
end
    assign data = (hexOut10*4'b1010)+hexOut1;
    assign out1 = hexOut1;
    assign out10 = hexOut10;
endmodule

//audio section
module audiostartRAM(inputData,outputData,address,writeEnable,clock);
    input [5:0] inputData;
    input [19:0] address;
        //10 bit address accounts for 1024 locations
        //10 bit address to account 785 locations 0 - 784

    input writeEnable, clock;
    output reg [5:0] outputData;

    reg [5:0] soundRAM [377974:0] /* synthesis ram_init_file = "audiostart.mif" */;
        //0 should be the first bit
        //1 is first pixel in the img
        //784 is the last pixel

    

    always @(posedge clock) begin
        if(writeEnable)begin
            soundRAM[address] <= inputData;
        end
        outputData <= soundRAM[address];
    end
endmodule

module csharp(inputData,outputData,address,writeEnable,clock);
    input [5:0] inputData;
    input [15:0] address;
        //10 bit address accounts for 1024 locations
        //10 bit address to account 785 locations 0 - 784

    input writeEnable, clock;
    output reg [5:0] outputData;

    reg [5:0] soundRAM [48278:0] /* synthesis ram_init_file = "csharp.mif" */;
        //0 should be the first bit
        //1 is first pixel in the img
        //784 is the last pixel

    

    always @(posedge clock) begin
        if(writeEnable)begin
            soundRAM[address] <= inputData;
        end
        outputData <= soundRAM[address];
    end
endmodule

module RateDivideraudio(Clock, resetn, Enable, Enablecounter44100hz); //f
    input Clock;
    input resetn;
    output reg Enable;
    input Enablecounter44100hz;

    reg [10:0] Q = 0;

    always @(posedge Clock) begin
        if (!resetn)
            Q <= 0;

        else if (Q == 11'd1134) begin 
            Enable <= 1;
            Q <= 0;
        end

        else if (Enablecounter44100hz) begin
            Q <= Q + 11'd1;
            Enable <= 0;
        end
    end
endmodule

//create a counter which counts up to 1000 every enable

module countto377974(Clock, resetn, Enable, addressaudio); //for 3 seconds
    input Clock;
    input resetn;
    input Enable;
    output reg [19:0] addressaudio = 0;

    always @(posedge Clock) begin
        if (!resetn)
            addressaudio <= 0;

        else if (addressaudio == 19'd377974) begin 
            addressaudio <= 0;
        end

        else if (Enable) begin
            addressaudio <= addressaudio + 19'd1;
        end
    end
endmodule

module countto48279(Clock, resetn, Enable, addressaudio, done); //for 3 seconds
    input Clock;
    input resetn;
    input Enable;
    output reg done = 0;
    output reg [15:0] addressaudio = 0;

    always @(posedge Clock) begin
        if (!resetn) begin
            addressaudio <= 0;
            done <= 0;
            
        end

        else if (addressaudio == 16'd48279) begin 
            addressaudio <= 0;
            done <= 1;
        end

        else if (Enable) begin
            addressaudio <= addressaudio + 16'd1;
            done <= 0;
        end
    end
endmodule

