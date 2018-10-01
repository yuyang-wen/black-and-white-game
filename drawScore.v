module drawScore (clk, resetn, enable, p1, p2, color, x, y);
input clk, enable;
input resetn;
input [2:0] p1;
input [2:0] p2;

output reg [7:0] x;
output reg [2:0] color;
output reg [6:0] y;
reg [5:0] Q;

always @(posedge clk) begin
    if (!resetn) begin
        Q <= 6'b0;
    end
    else if (Q == 6'd60) begin
        Q <= 6'b0;
    end
    else if (enable) begin
        Q <= Q + 6'd1;
    end    
end

always @(*) begin
	if (Q < 6'd29 ) begin
		color = 3'b111;
	end
	else if (Q >= 6'd30) begin
		color = 3'b000;
	end
	else color = 3'b0;
end

always @(*) begin
	if (Q < 6'd29) begin
		x = Q;
	end
	else if (Q >= 6'd30) begin
		x = Q + 8'd100;
	end
	else x = 3'b0;
end

always @(*) begin
	if (Q < 6'd29) begin
		case (p1)
		3'b000: y = 7'd1;
		3'b001: y = 7'd24;
		3'b010: y = 7'd48;
		3'b011: y = 7'd72;
		3'b100: y = 7'd96;
		default: y = 0;
		endcase
	end
	else if (Q >= 6'd30) begin
		case (p2)
		3'b000: y = 7'd1;
		3'b001: y = 7'd24;
		3'b010: y = 7'd48;
		3'b011: y = 7'd72;
		3'b100: y = 7'd96;
		default: y = 0;
		endcase
	end
	else y = 3'b0;
end

endmodule
