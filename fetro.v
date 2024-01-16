module fetro (CLOCK_50,
               SW,
               LEDR,
               KEY,
               HEX0,
               HEX1,
               
					AUD_ADCDAT,
               AUD_BCLK,
               AUD_ADCLRCK,
               AUD_DACLRCK,
               FPGA_I2C_SDAT,
               AUD_XCK,
               AUD_DACDAT,
               FPGA_I2C_SCLK,
               
					PS2_CLK,
               PS2_DAT,       // On Board Keys
               VGA_CLK,       // VGA Clock
               VGA_HS,        // VGA H_SYNC
               VGA_VS,        // VGA V_SYNC
               VGA_BLANK_N,   // VGA BLANK
               VGA_SYNC_N,    // VGA SYNC
               VGA_R,         // VGA Red[9:0]
               VGA_G,         // VGA Green[9:0]
               VGA_B);        // VGA Blue[9:0]
    
   
    
    output [6:0] HEX0, HEX1;
    
    input        CLOCK_50;                     //  50 MHz
    input  [3:0] KEY;
    input  [9:0] SW;
    output [7:0] LEDR;
    
    inout				PS2_CLK;
    inout				PS2_DAT;
    
    output          VGA_CLK;                //  VGA Clock
    output          VGA_HS;                 //  VGA H_SYNC
    output          VGA_VS;                 //  VGA V_SYNC
    output          VGA_BLANK_N;            //  VGA BLANK
    output          VGA_SYNC_N;             //  VGA SYNC
    output  [7:0]   VGA_R;                  //  VGA Red[7:0] Changed from 10 to 8-bit DAC
    output  [7:0]   VGA_G;                  //  VGA Green[7:0]
    output  [7:0]   VGA_B;                  //  VGA Blue[7:0]
    
    // Create the colour, x, y and writeEn wires that are inputs to the controller.
    
    wire [2:0] colour;
    wire [7:0] x;
    wire [6:0] y;
    wire writeEn;
    
    wire resetn;
    assign resetn = KEY[0];
    
    vga_adapter VGA
    (
    .resetn(resetn),
    .clock(CLOCK_50),
    .colour(colour),
    .x(x),
    .y(y),
    .plot(writeEn),
    
    .VGA_R(VGA_R),
    .VGA_G(VGA_G),
    .VGA_B(VGA_B),
    .VGA_HS(VGA_HS),
    .VGA_VS(VGA_VS),
    .VGA_BLANK(VGA_BLANK_N),
    .VGA_SYNC(VGA_SYNC_N),
    .VGA_CLK(VGA_CLK) 
    );
    
    defparam VGA.RESOLUTION              = "160x120";
    defparam VGA.MONOCHROME              = "FALSE";
    defparam VGA.BITS_PER_COLOUR_CHANNEL = 1;
    defparam VGA.BACKGROUND_IMAGE        = "black.mif";
            
    input				AUD_ADCDAT;

    // Bidirectionals
    inout				AUD_BCLK;
    inout				AUD_ADCLRCK;
    inout				AUD_DACLRCK;
    inout				FPGA_I2C_SDAT;

    // Outputs
    output				AUD_XCK;
    output				AUD_DACDAT;
    output				FPGA_I2C_SCLK;

    // Internal Wires
    wire				audio_in_available;
    wire		[31:0]	leftIn;
    wire		[31:0]	rightIn;
    wire				readIn;

    wire				audio_out_allowed;
    wire		[31:0]	leftOut;
    wire		[31:0]	rightOut;
    wire				writeOut;

    // Internal Registers

    reg [18:0] delay_count;
    wire [18:0] delay;

    reg send;

    always @(posedge CLOCK_50)
        if(delay_count == delay) begin
            delay_count <= 0;
            send <= !send;
        end else delay_count <= delay_count + 1;

    assign delay = {15'd3000};

    wire [31:0] sound = send ? 32'd10000000 : 0;

    assign readIn			= audio_in_available     & audio_out_allowed;
    assign leftOut	                = leftIn  + sound;
    assign rightOut	                = rightIn + sound;
    assign writeOut			= audio_in_available     & audio_out_allowed;
 
        Audio_Controller Audio_Controller 
        (
            // Inputs
            .CLOCK_50				    (CLOCK_50),
            .reset						(~KEY[0]),

            .clear_audio_in_memory		(),
            .read_audio_in				(readIn),
            .clear_audio_out_memory		(),
            .left_channel_audio_out		(leftOut),
            .right_channel_audio_out	(rightOut),
            .write_audio_out			(writeOut),

            .AUD_ADCDAT					(AUD_ADCDAT),

            // Bidirectionals
            .AUD_BCLK					(AUD_BCLK),
            .AUD_ADCLRCK				(AUD_ADCLRCK),
            .AUD_DACLRCK				(AUD_DACLRCK),

            // Outputs
            .audio_in_available			(audio_in_available),
            .left_channel_audio_in		(leftIn),
            .right_channel_audio_in		(rightIn),

            .audio_out_allowed			(audio_out_allowed),

            .AUD_XCK					(AUD_XCK),
            .AUD_DACDAT					(AUD_DACDAT)
        );


        // PS2 KEYBOARD
        wire		[7:0] ps2_key_data;
        wire		      ps2_key_pressed;
        
        PS2_Controller PS2
        (
            // Inputs
            .CLOCK_50           (CLOCK_50),
            .reset				(!KEY[0]),
            
            // Bidirectionals
            .PS2_CLK			(PS2_CLK),
            .PS2_DAT			(PS2_DAT),
            
            // Outputs
            .received_data		(ps2_key_data),
            .received_data_en	(ps2_key_pressed)
        );
        
        Top entry
        (
            .clk(CLOCK_50),
            
            // VGA
            .VGA_X(x),
            .VGA_Y(y),
            .VGA_COLOR(colour),
            .plot(writeEn),
            
            // FPGA
            .LEDR(LEDR[7:0]),
            .SW(SW[9:0]),
            .h0(HEX0),
            .h1(HEX1),
            
            
            // PS2 Keyboard
            .data(ps2_key_data),
            .keyen(ps2_key_pressed),

            // Audio    
            .audioLIn(),
            .audioRIn(),
            .audioLOut(),
            .audioROut()
        );
        
        endmodule
        
        
        module Top (clk, VGA_X, VGA_Y, VGA_COLOR, plot, LEDR, SW, data, keyen, h0, h1, audioLIn, audioRIn, audioLOut, audioROut);
            
            // Clock
            input  wire       clk;         // DE-series 50 MHz clock signal
            
            // VGA
            output wire [7:0] VGA_X;      // "VGA" column
            output wire [6:0] VGA_Y;      // "VGA" row
            output wire [2:0] VGA_COLOR;  // "VGA pixel" colour (0-7)
            output wire       plot;       // "Pixel" is drawn when this is pulsed
            
            // KEYBOARD
            input wire        keyen;
            input wire  [7:0] data;
            
            // FPGA
            input wire  [9:0] SW;
            output wire [9:0] LEDR;
            
            // AUDIO
            input wire		[31:0]	audioLIn;
            input wire		[31:0]	audioRIn;
            output wire		[31:0]	audioLOut;
            output wire		[31:0]	audioROut;
            
            // input wire [3:0] KEY;
            output wire [6:0] h0;
            output wire [6:0] h1;
            
            // DIMENSIONS
            parameter SECOND = 'd10000000;
            
            localparam X_SCREEN_DIM = 8'd160,
            Y_SCREEN_DIM = 7'd120,
            
            // Colour
            BLACK = 3'b000,
            BLUE = 3'b001,
            GREEN = 3'b010,
            CYAN = 3'b011,
            RED = 3'b100,
            MAGENTA = 3'b101,
            YELLOW = 3'b110,
            WHITE = 3'b111,
            
            // Board Dimensions
            YDIM = 'd20,
            XDIM = 'd10;
            
            // Game
            reg         [2:0] color = 0;
            reg         [3:0] mino [3:0];
            reg         [0:9] game [19:0];
            
            initial
            begin
                // BOARD
                game[0][0:9]  <= 'b000000000;
                game[1][0:9]  <= 'b000000000;
                game[2][0:9]  <= 'b000000000;
                game[3][0:9]  <= 'b000000000;
                game[4][0:9]  <= 'b000000000;
                game[5][0:9]  <= 'b000000000;
                game[6][0:9]  <= 'b000000000;
                game[7][0:9]  <= 'b000000000;
                game[8][0:9]  <= 'b000000000;
                game[9][0:9]  <= 'b000000000;
                game[10][0:9] <= 'b000000000;
                game[11][0:9] <= 'b000000000;
                game[12][0:9] <= 'b000000000;
                game[13][0:9] <= 'b000000000;
                game[14][0:9] <= 'b000000000;
                game[15][0:9] <= 'b000000000;
                game[16][0:9] <= 'b000000000;
                game[17][0:9] <= 'b000000000;
                game[18][0:9] <= 'b000000000;
                game[19][0:9] <= 'b000000000;
            end
            
            reg        [7:0] outX;
            reg        [6:0] outY;
            reg        [3:0] sX = 3'b0;
            reg        [6:0] sY = 6'd0;
            
            // Counters
            reg        [10:0] cSq   = 0;
            reg        [28:0] timeC = 0;
            
            reg                        clear;
            reg                        block = 1;
            reg                        turn  = 0;
            
            reg             [3:0] intersect;
            reg             [16:0] available;
            
            reg			    [7:0] keyData;
            reg             [5:0] piece = 0;
            
            
            localparam  SET = 4'd0,
            PLOT = 4'd1,
            CLEAR = 4'd2,
            INPUT = 4'd3;
            
            reg shouldPlot;
            reg isDone;
            reg [3:0] current, next;
            reg [3:0] rot = 0;
            reg [7:0] score = 0;
            
            reg         [4:0] scale = 0;
            

            always@(*)
                begin: state_table
                case (current)
                    SET:
                    begin
                        next = PLOT;
                    end
                    PLOT:
                    begin
                        next = (isDone && timeC >= SECOND) ? CLEAR : PLOT;
                    end
                    CLEAR:
                    begin
                        next = (isDone) ? INPUT : CLEAR;
                    end
                    INPUT:
                    begin
                        next = (rotCount >= SECOND) ? SET : INPUT;
                    end
                    
                    default: next = SET;
                    
                endcase
                end // state_table
            
            
            always @(*)
                begin: control_signals
            
            case (current)
                SET:
                begin
                    shouldPlot = 0;
                end
                PLOT:
                begin
                    shouldPlot = 1;
                end
                CLEAR:
                begin
                    //            color = BLACK;
						  rotCount = 0;
                    shouldPlot          = 1;
                end
                INPUT:
                begin
                    shouldPlot = 0;
                end
            endcase
            end
            
            wire [18:0] delay = {15'd3000};
            reg  [18:0] delay_count;
            
            reg send;
            wire [31:0] sound = send ? 32'd10000000 : 0;
            
            assign audioLOut	 = audioLIn  + sound;
            assign audioROut	 = audioRIn  + sound;
            
			reg [18:0] rotCount = 0;
				
            // ENTRY
            always@(posedge clk)
            begin
					
				
                if (SW[0] != 0)
                begin
                
                keyData <= data;
                current <= next;
                
                // Update Counters.
					 		
			
                timeC <= timeC + 1;
                
                
				   rotCount <= rotCount + 1;
					
					if (rotCount >= SECOND)
					begin
						rotCount <= 0;
					end
						
				  if (timeC >= SECOND && !intersect)
                begin
                    color <= BLACK;
                end
                else
                begin
						  if (piece == 0)
                    begin
                        color  <= MAGENTA; // T

                            if (keyData == 'h75 && current == INPUT)
                            begin
                                rot <= rot + 1;
                            end
								
                            if (rot > 3)
                            begin
                                rot <= 0;
                            end

                            if (rot == 0)
                            begin                             
                                mino[0][3:0] <= 4'b0000;
                                mino[1][3:0] <= 4'b0000;
                                mino[2][3:0] <= 4'b0100;
                                mino[3][3:0] <= 4'b1110;
                            end
                            else if (rot == 1)
                            begin                         
                                mino[0][3:0] <= 4'b0000;
                                mino[1][3:0] <= 4'b1000;
                                mino[2][3:0] <= 4'b1100;
                                mino[3][3:0] <= 4'b1000;
                            end
                            else if (rot == 2)
                            begin                         
                                mino[0][3:0] <= 4'b0000;
                                mino[1][3:0] <= 4'b0000;
                                mino[2][3:0] <= 4'b1110;
                                mino[3][3:0] <= 4'b0100;
                            end
                            else if (rot == 3)
                            begin                                 
                                mino[0][3:0] <= 4'b0000;
                                mino[1][3:0] <= 4'b0100;
                                mino[2][3:0] <= 4'b1100;
                                mino[3][3:0] <= 4'b0100;
                            end

                    end
                    else if (piece == 1)
                    begin
                        color        <= GREEN;  // S
//

                                if (keyData == 'h75)
                                begin
                                    rot <= rot + 1;
                                end
                                if (rot > 1)
                                begin
                                    rot <= 0; 
                                end

                                if (rot == 0)
                                begin                   
                                    mino[0][3:0] <= 4'b0000;
                                    mino[1][3:0] <= 4'b1000;
                                    mino[2][3:0] <= 4'b1100;
                                    mino[3][3:0] <= 4'b0100;
                                end
                                else if (rot == 1)
                                begin                   
                                    mino[0][3:0] <= 4'b0000;
                                    mino[1][3:0] <= 4'b0000;
                                    mino[2][3:0] <= 4'b0110;
                                    mino[3][3:0] <= 4'b1100;
                                end

                    end
                        else if (piece == 2)
                        begin
                             color        <= CYAN; // I

                                if (keyData == 'h75)
                                begin
                                    rot <= rot + 1;
                                end
                                if (rot > 1)
                                begin
                                    rot <= 0;
                                end

                                if (rot == 0)
                                begin                             
                                    mino[0][3:0] <= 4'b1000;
                                    mino[1][3:0] <= 4'b1000;
                                    mino[2][3:0] <= 4'b1000;
                                    mino[3][3:0] <= 4'b1000;
                                end
                                else if (rot == 1)
                                begin                   
                                    mino[0][3:0] <= 4'b0000;
                                    mino[1][3:0] <= 4'b0000;
                                    mino[2][3:0] <= 4'b0000;
                                    mino[3][3:0] <= 4'b1111;
                                end
                        end
                        else if (piece == 3)
                        begin
                        color        <= RED; // Z

                                if (keyData == 'h75)
                                begin
                                    rot <= rot + 1;
                                end
                                if (rot > 1)
                                begin
                                    rot <= 0;
                                end

                                if (rot == 0)
                                begin                   
                                    mino[0][3:0] <= 4'b0000;
                                    mino[1][3:0] <= 4'b0100;
                                    mino[2][3:0] <= 4'b1100;
                                    mino[3][3:0] <= 4'b1000;
                                end
                                else if (rot == 1)
                                begin                   
                                    mino[0][3:0] <= 4'b0000;
                                    mino[1][3:0] <= 4'b0000;
                                    mino[2][3:0] <= 4'b1100;
                                    mino[3][3:0] <= 4'b0110;
                                end
                        end
                        else if (piece == 4)
                        begin
                        color        <= WHITE; // L
                      

                                if (keyData == 'h75)
                                begin
                                    rot <= rot + 1;
                                end
                                if (rot > 3)
                                begin
                                    rot <= 0;
                                end
                                
                                // Rotations.
                                if (rot == 0)
                                begin        
                                    mino[0][3:0] <= 4'b0000;
                                    mino[1][3:0] <= 4'b1000;
                                    mino[2][3:0] <= 4'b1000;
                                    mino[3][3:0] <= 4'b1100;
                                end
                                else if (rot == 1)
                                begin
                                    mino[0][3:0] <= 4'b0000;
                                    mino[1][3:0] <= 4'b0000;
                                    mino[2][3:0] <= 4'b1110;
                                    mino[3][3:0] <= 4'b1000;
                                end
                                else if (rot == 2)
                                begin
                                    mino[0][3:0] <= 4'b0000;
                                    mino[1][3:0] <= 4'b1100;
                                    mino[2][3:0] <= 4'b0100;
                                    mino[3][3:0] <= 4'b0100;
                                end
                                else if (rot == 3)
                                begin
                                    mino[0][3:0] <= 4'b0000;
                                    mino[1][3:0] <= 4'b0000;
                                    mino[2][3:0] <= 4'b0010;
                                    mino[3][3:0] <= 4'b1110;
                                end
//                            end
                        end
                        else if (piece == 5) // J
                        begin
                            color <= BLUE;
                                if (keyData == 'h75)
                                begin
                                    rot <= rot + 1;
                                end
                                if (rot > 3)
                                begin
                                    rot <= 0;
                                end

                                if (rot == 0)
                                begin                   
                                    mino[0][3:0] <= 4'b0000;
                                    mino[1][3:0] <= 4'b0100;
                                    mino[2][3:0] <= 4'b0100;
                                    mino[3][3:0] <= 4'b1100;
                                end
                                else if (rot == 1)
                                begin
                                    mino[0][3:0] <= 4'b0000;
                                    mino[1][3:0] <= 4'b0000;
                                    mino[2][3:0] <= 4'b1000;
                                    mino[3][3:0] <= 4'b1110;
                                end
                                else if (rot == 2)
                                begin
                                    mino[0][3:0] <= 4'b0000;
                                    mino[1][3:0] <= 4'b1100;
                                    mino[2][3:0] <= 4'b1000;
                                    mino[3][3:0] <= 4'b1000;
                                end
                                else if (rot == 3)
                                begin
                                    mino[0][3:0] <= 4'b0000;
                                    mino[1][3:0] <= 4'b0000;
                                    mino[2][3:0] <= 4'b1110;
                                    mino[3][3:0] <= 4'b0010;
                                end
                        end
                        else if (piece == 6) // O
                        begin
                            color        <= YELLOW; 
                            mino[0][3:0] <= 4'b0000;
                            mino[1][3:0] <= 4'b0000;
                            mino[2][3:0] <= 4'b1100;
                            mino[3][3:0] <= 4'b1100;

                            // No rotation for o piece
                        end
                        end
                    end
                      
                        // cSq <= cSq + 1;
                        
                        // Print tiles.
                        if (mino[cSq[3:2]][3 - cSq[1:0]] == 'b1)
                        begin
                            // outY <= sY + cSq[3:2];
                            // outX <= sX + cSq[1:0];
                            outY    <= sY * 4 + (4) * cSq[3:2] +  scale[3:2];
                            outX    <= sX * 4 + (4) * cSq[1:0] +  scale[1:0];
                        end
                        else
                        begin
                            outY <= -1;
                            outX <= -1;
                        end
                        
                        if (scale < 16)
                        begin
                            scale <= scale + 1;
                        end
                        else
                        begin
                            scale <= 0;
                            cSq   <= cSq + 1;
                        end
                        
                        // Y-Bounding.
                        if (sY + 4 < 20)
                        begin
                            // Intersection.
                            intersect <= (((mino[0][3:0] & game[sY+1][sX+:4])
                            | (mino[1][3:0] & game[sY+1+1][sX+:4])
                            | (mino[2][3:0] & game[sY+2+1][sX+:4])
                            | (mino[3][3:0] & game[sY+3+1][sX+:4])) > 0);
                        end
                        else
                        begin
                            intersect <= 1;
                        end
                        
                        
                        // if (intersect || game[sY+1][sX] ! = 0)
                        if (intersect &&  available != -1)
                        begin
                            // Intersecting?
                            game[available + 0][sX+:4] <= game[available + 0][sX+:4] | mino[0][3:0];
                            game[available + 1][sX+:4] <= game[available + 1][sX+:4] | mino[1][3:0];
                            game[available + 2][sX+:4] <= game[available + 2][sX+:4] | mino[2][3:0];
                            game[available + 3][sX+:4] <= game[available + 3][sX+:4] | mino[3][3:0];
                        end
                        else
                        begin
                            available = sY;
                        end
                        
                                 
                        // Finished drawing.
                        if (cSq > 32)
                        begin
                            isDone <= 1;
                            
                            // Finished clearing.
                            if (color == BLACK || intersect)
                            begin
                                timeC <= 0;
                                
                                // CHANGE X.
                                // if ((keyData) == 'hE0)
                                // begin
                                // if (keyData == 'he0)
                                // begin
                                //     turn <= 1;
                                // end
                                keyData <= data;
                                
                                
                                if (keyData == 'h74 && sX < 6)
                                begin
                                    turn <= 0;
                                    
                                    if (game[sY+1][sX+1] == 0)
                                    begin
                                        sX <= sX + 1;
                                    end
                                end
                                else if (keyData == 'h6b && sX > 0)
                                begin
                                    turn <= 0;
                                    
                                    if (game[sY+1][sX-1] == 0)
                                    begin
                                        sX <= sX - 1;
                                    end
                                end
                                    // end
                                    
                                    // CHANGE Y.
                                    if ((sY == YDIM - 1) || intersect)
                                    begin
                                        sY    <= 0;
                                        sX    <= 0;
                                        rot   <= 0;
                                        score <= score + 1;
                                        piece <= piece + 1;
                                        
                                        available <= -1;
                                        
                                        // AUDIO
                                        delay_count <= 0;
                                        send        <= 1;
                                        
                                        if (piece > 6)
                                        begin
                                            piece <= 0;
                                        end
                                    end
                                    else
                                    begin
													
                                        sY <= sY + 1;
                                    end
                                    
                                    end
                                    
                                    cSq <= 0;
                                    end
                                else
                                begin
                                    isDone <= 0;
                                end
										  
										  
                            end
                            
//									 module scoreStore (address, clock, data, wren, q);
									 scoreStore M1 ('b0, clk, score, 'b1, q);
									 									
									assign LEDR[9:0]   = score;
                            
                            //    wire[6:0] primaryDig;
                            //    wire[6:0] secondaryDig;
                            
                            hex_decoder U1 (score[3:0], h0); // First
                            hex_decoder U2 (score[7:4], h1); // Second
                            
                            //    assign h0 = secondaryDig;
                            //    assign h1 = primaryDig;
                            
                            
                            assign VGA_X     = outX;
                            assign VGA_Y     = outY;
                            assign VGA_COLOR = color;
                            assign plot      = shouldPlot;
                            
                            endmodule

module hex_decoder(c, display);
    
    input [3:0] c;
    output [6:0] display;

    assign display[0] = 
    ~((~c[3] & ~c[2] & ~c[1] & ~c[0]) |
    (~c[3] & ~c[2] & c[1] & ~c[0]) |
    (~c[3] & ~c[2] & c[1] & c[0]) |
    (~c[3] & c[2] & ~c[1] & c[0]) |
    (~c[3] & c[2] & c[1] & ~c[0]) |
    (~c[3] & c[2] & c[1] & c[0]) |
    (c[3] & ~c[2] & ~c[1] & ~c[0]) |
    (c[3] & ~c[2] & ~c[1] & c[0]) |
    (c[3] & ~c[2] & c[1] & ~c[0]) |
    (c[3] & c[2] & ~c[1] & ~c[0]) |
    (c[3] & c[2] & c[1] & ~c[0]) |
    (c[3] & c[2] & c[1] & c[0]));
    
    // m(1,2,3,4,7,8,9,A,d)
    assign display[1] = 
    ~((~c[3] & ~c[2] & ~c[1] & ~c[0]) |
    (~c[3] & ~c[2] & ~c[1] & c[0]) |
    (~c[3] & ~c[2] & c[1] & ~c[0]) |
    (~c[3] & ~c[2] & c[1] & c[0]) |
    (~c[3] & c[2] & ~c[1] & ~c[0]) |
    (~c[3] & c[2] & c[1] & c[0]) |
    (c[3] & ~c[2] & ~c[1] & ~c[0]) |
    (c[3] & ~c[2] & ~c[1] & c[0]) |
    (c[3] & ~c[2] & c[1] & ~c[0]) |
    (c[3] & c[2] & ~c[1] & c[0]));
    
    // m(0,1,3,4,5,6,7,8,9,A,b,d)
    assign display[2] = 
    ~((~c[3] & ~c[2] & ~c[1] & ~c[0]) |
    (~c[3] & ~c[2] & ~c[1] & c[0]) |
    (~c[3] & ~c[2] & c[1] & c[0]) |
    (~c[3] & c[2] & ~c[1] & ~c[0]) |
    (~c[3] & c[2] & ~c[1] & c[0]) |
    (~c[3] & c[2] & c[1] & ~c[0]) |
    (~c[3] & c[2] & c[1] & c[0]) |
    (c[3] & ~c[2] & ~c[1] & ~c[0]) |
    (c[3] & ~c[2] & ~c[1] & c[0]) |
    (c[3] & ~c[2] & c[1] & ~c[0]) |
    (c[3] & ~c[2] & c[1] & c[0]) |
    (c[3] & c[2] & ~c[1] & c[0]));
    
    // m(0,2,3,5,6,8,9,b,C,d,E)
    assign display[3] = 
    ~((~c[3] & ~c[2] & ~c[1] & ~c[0]) |
    (~c[3] & ~c[2] & c[1] & ~c[0]) |
    (~c[3] & ~c[2] & c[1] & c[0]) |
    (~c[3] & c[2] & ~c[1] & c[0]) |
    (~c[3] & c[2] & c[1] & ~c[0]) |
    (c[3] & ~c[2] & ~c[1] & ~c[0]) |
    (c[3] & ~c[2] & ~c[1] & c[0]) |
    (c[3] & ~c[2] & c[1] & c[0]) |
    (c[3] & c[2] & ~c[1] & ~c[0]) |
    (c[3] & c[2] & ~c[1] & c[0]) |
    (c[3] & c[2] & c[1] & ~c[0]));
    
    // m(0,2,6,8,A,b,C,d,E,F)
    assign display[4] = 
    ~((~c[3] & ~c[2] & ~c[1] & ~c[0]) |
    (~c[3] & ~c[2] & c[1] & ~c[0]) |
    (~c[3] & c[2] & c[1] & ~c[0]) |
    (c[3] & ~c[2] & ~c[1] & ~c[0]) |
    (c[3] & ~c[2] & c[1] & ~c[0]) |
    (c[3] & ~c[2] & c[1] & c[0]) |
    (c[3] & c[2] & ~c[1] & ~c[0]) |
    (c[3] & c[2] & ~c[1] & c[0]) |
    (c[3] & c[2] & c[1] & ~c[0]) |
    (c[3] & c[2] & c[1] & c[0]));
    
    // m(0,4,5,6,8,9,A,b,C,E,F)
    assign display[5] = 
    ~((~c[3] & ~c[2] & ~c[1] & ~c[0]) |
    (~c[3] & c[2] & ~c[1] & ~c[0]) |
    (~c[3] & c[2] & ~c[1] & c[0]) |
    (~c[3] & c[2] & c[1] & ~c[0]) |
    (c[3] & ~c[2] & ~c[1] & ~c[0]) |
    (c[3] & ~c[2] & ~c[1] & c[0]) |
    (c[3] & ~c[2] & c[1] & ~c[0]) |
    (c[3] & ~c[2] & c[1] & c[0]) |
    (c[3] & c[2] & ~c[1] & ~c[0]) |
    (c[3] & c[2] & c[1] & ~c[0]) |
    (c[3] & c[2] & c[1] & c[0]));
    
    // m(2,3,4,5,6,8,9,A,b,d,E,F)
    assign display[6] = 
    ~((~c[3] & ~c[2] & c[1] & ~c[0]) |
    (~c[3] & ~c[2] & c[1] & c[0])  |
    (~c[3] & c[2] & ~c[1] & ~c[0]) |
    (~c[3] & c[2] & ~c[1] & c[0])  |
    (~c[3] & c[2] & c[1] & ~c[0])  |
    (c[3] & ~c[2] & ~c[1] & ~c[0]) |
    (c[3] & ~c[2] & ~c[1] & c[0]) |
    (c[3] & ~c[2] & c[1] & ~c[0]) |
    (c[3] & ~c[2] & c[1] & c[0])  |
    (c[3] & c[2] & ~c[1] & c[0])  |
    (c[3] & c[2] & c[1] & ~c[0])  |
    (c[3] & c[2] & c[1] & c[0]));
    
endmodule