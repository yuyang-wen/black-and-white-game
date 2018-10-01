    
module datapathAudio (
    input CLOCK_50, 
    input resetn,
    input e_csharp, 
    input resetcsharp, 

    output [31:0] sound;
    );

    reg signed [31:0] audiostart;
    reg signed [31:0] csharp;

    always @(*) begin
        if (1)begin
            audiostart = {audioamplitude, zero };
        end

        else begin
            audiostart = 32'b0;
        end

        if (e_csharp || (done == 0)) begin
            csharp =  {audioamplitudecsharp, zero };
        end

        else begin
            csharp = 32'b0;
        end

    end 

    assign sound =  csharp + audiostart;

    wire signed [5:0] audioamplitude;
    wire signed [5:0] audioamplitudecsharp;
    wire signed [25:0] zero;
    wire done;
    // wire signed [25:0] zerocsharp;
    assign zero = 26'b0;
    // assign zerocsharp = 
    wire [19:0] addressaudio;
    wire [15:0] addressaudiocsharp;

    RateDivideraudio r10(CLOCK_50, resetn, Enable, 1); 
    countto377974 c0(CLOCK_50, resetn, Enable, addressaudio); 

    always @(posedge CLOCK_50) begin
        if (done == 1) begin
            // reset
            Enablecounter44100hz <= 0;
        end
        else if (done == 0) begin
            Enablecounter44100hz <= e_csharp;
        end
    end

    reg Enablecounter44100hz;

    RateDivideraudio r11(CLOCK_50, resetn, Enablecsharp, Enablecounter44100hz); 
    countto48279 c2(CLOCK_50, ~resetcsharp, Enablecsharp, addressaudiocsharp, done); 

    audiostartRAM s2(0,audioamplitude,addressaudio,0,CLOCK_50);
    csharp c1(0,audioamplitudecsharp,addressaudiocsharp,0,CLOCK_50);

    endmodule