/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                         //
//                                                                                                         //
//    -+++==-          -*****+                                                                             //
//     *%###-          .@@@@@-                                                                             //
//      +*++=.         *@@@@-                                                                              //
//       ====-        +%%%%=                                                                               //
//       .====-      -####=  :++++++++++++   -++++++++=-.         :=+****+-.      .+++:       =++=         //
//        .=+++=    :+++*-   -%%%%%%%%%%%#   +%%%%%%%%%%%#-     =#%%%%##%%%%*:    :%%%%=      #%%#         //
//         :***#=  .====-                    =%%%:   .:*%%%-  .#%%%+:   .-*%%%+   :%%%%%*:    *%%#         //
//          -%###=.===-:                     =%%%.     .%%%*  *%%%:        +%%%-  :%%%%%%%=   *%%#         //
//           =%%%#**++-      -%%%%%%%%%%%#   =%%%.     =%%%= .%%%+          #%%*  :%%%==%%%*. *%%#         //
//            =@%%###+       :++++++++++++   =%%%*****#%%%+  .%%%+         .%%%*  :%%%- :#%%%=#%%#         //
//             +@%%%#.                       =%%%####%%%*.    *%%%-        *%%%:  :%%%=   =%%%%%%#         //
//              *@@%:         ............   =%%%:   =%%%=     *%%%*-:..:=#%%%-   :%%%=    :#%%%%#         //
//               #@:         =@%%%%%%%%%%%   +%%%.    :#%@*.    -*%%%%%%%%%#+.    :%%@=      +%%%#         //
//               .:          .------------   :---      .--=:      .:-=+==-:        ---.       :---         //
//                                                                                                         //
//     :- :: -: : :: -.. :- ::   : -... - .-...     .. -. - ::.. -: :.  . -: -: :: : :- : : :: -:          //
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Company: Veron Group                                                                                    //
// Author:  RTL DN TEAM                                                                                    //
//                                                                                                         //
// Create Date:    20/09/2025                                                                              //
// Module Name:    RGB_DETECTOR_INTERFACE                                                                  //
// Revision History:                                                                                       //
// Revision 0.01 - File Created (Author)                                                                   //
// Revision 0.02 - modified to work with Cyclone10 board                                                   //
// Property of Veron Group                                                                                 //
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

module RGB_DETECTOR_INTERFACE (
    input wire clk,      // 12MHz system clock
    input wire rst_n,    // Active low reset
    inout wire SCL,      // I2C clock
    inout wire SDA,      // I2C data
 
	// Color detection outputs
    output reg RED,     // Red detected
    output reg GREEN,   // Green detected
    output reg BLUE,    // Blue detected
    output reg WHITE    // Unknown color
);

//======================== Status & Flags ========================
// data_valid: 1 for one clk when new RGBC data is ready
reg        data_valid;
wire       busy;              // I2C core is busy (1) or idle (0)

//===================== Sensor Data (16-bit) =====================
// Final 16-bit values after we join high/low bytes
reg [15:0] clear;
reg [15:0] red;
reg [15:0] green;
reg [15:0] blue;

// enable: start one I2C action (write or read)
// read_write: 0 = write, 1 = read
reg             enable;
reg             read_write;

// 7-bit I2C device address (TCS3472 = 0x29)
reg     [6:0]   device_address = 7'h29;

// I2C register address (command + addr for TCS3472)
reg     [7:0]   register_address;

// Data to send (write) and data received (read)
reg     [7:0]   mosi_data;      // master -> sensor
wire    [7:0]   miso_data;      // sensor -> master

//================ Timing / Delay Helpers ========================
// delay_counter: wait times (e.g., 2.4 ms for power-on or ATIME)
reg     [23:0]  delay_counter;

//=============== Byte Buffers (Temporary 8-bit) =================
// We first read/write low and high bytes, then join them to 16-bit
reg     [7:0]   clear_l,  clear_h;
reg     [7:0]   red_l,    red_h;
reg     [7:0]   green_l,  green_h;
reg     [7:0]   blue_l,   blue_h;

//============== Color Classifier (Output Wires) =================
// These are the results from the color-detect module
wire            color_is_red;
wire            color_is_green;
wire            color_is_blue;
wire            color_is_unknown;



// FSM states
typedef enum logic [4:0] {
    S_IDLE,
    S_WRITE_ENABLE,
    S_WAIT_PON,
    S_WRITE_ATIME,
    S_WRITE_GAIN,
    S_READ_CLEAR_L,
    S_READ_CLEAR_H,
    S_READ_RED_L,
    S_READ_RED_H,
    S_READ_GREEN_L,
    S_READ_GREEN_H,
    S_READ_BLUE_L,
    S_READ_BLUE_H,
    S_UPDATE_OUTPUT,
    S_DONE
} state_t;
state_t state;


//===========================================================================
//=============================== I2C_INTERFACE =============================
//===========================================================================
I2C_INTERFACE #(
    .NUMBER_OF_DATA_BYTES(1),
    .NUMBER_OF_REGISTER_BYTES(1),
    .ADDRESS_WIDTH(7),
    .CHECK_FOR_CLOCK_STRETCHING(1),
    .CLOCK_STRETCHING_MAX_COUNT(1000)
) i2c_master_inst (
    .clock              (clk),
    .reset_n            (rst_n),
    .enable             (enable),
    .read_write         (read_write),
    .device_address     (device_address),
    .register_address   (register_address),
    .pll_clk_300k       (pll_clk_100k),
    .pll_locked         (pll_locked),  
    .mosi_data          (mosi_data),
    .miso_data          (miso_data),
    .busy               (busy),
    .scl                (SCL),
    .sda                (SDA)
);


//===========================================================================
//=============================== CYCLONE PLL ===============================
//===========================================================================
pll my_pll (
.inclk0                 (clk),
.c0                     (pll_clk_100k),
.locked                 (pll_locked)
);


//===========================================================================
//=============================== HANDLING_COLOR ============================
//===========================================================================
HANDLING_COLOR color_detect (
    .clk                (clk),
    .rst_n              (rst_n),
    .clear_data         (clear),
    .red_data           (red),
    .green_data         (green),
    .blue_data          (blue),
    .data_valid         (data_valid),
    .BLUE               (color_is_blue),
    .GREEN              (color_is_green),
    .RED                (color_is_red),
    .WHITE              (color_is_unknown)
);


assign RED      = color_is_red;
assign GREEN    = color_is_green;
assign BLUE     = color_is_blue;
assign WHITE    = color_is_unknown;


//===========================================================================
//=============================== RGB_CONTROLLER ============================
//===========================================================================
always @(posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        state           <= S_IDLE;
        enable          <= 0;
        red             <= 0;
        green           <= 0;
        blue            <= 0;
        clear           <= 0;
        data_valid      <= 0;
        delay_counter   <= 0;
        // Reset temporary registers
        clear_l <= 0; clear_h   <= 0;
        red_l   <= 0; red_h     <= 0;
        green_l <= 0; green_h   <= 0;
        blue_l  <= 0; blue_h    <= 0;
    end else begin
        enable          <= 0; // Default disable
        data_valid      <= 0; // Default no valid data
        
        case (state)
            S_IDLE: begin
                if (!busy) begin
                    // Enable sensor (PON + AEN)
                    register_address    <= 8'h80;       // ENABLE reg
                    mosi_data           <= 8'h03;       // PON | AEN
                    read_write          <= 0;
                    enable              <= 1;
                    delay_counter       <= 24'd28_800;  // 2.4ms delay
                    state               <= S_WAIT_PON;
                end
            end
            
            S_WAIT_PON: begin
                if (delay_counter > 0) begin
                    delay_counter       <= delay_counter - 1;
                end else begin
                    // Set integration time
                    register_address    <= 8'h81; // ATIME reg
                    mosi_data           <= 8'hFF; // 2.4ms
                    read_write          <= 0;
                    enable              <= 1;
                    state               <= S_WRITE_ATIME;
                end
            end
            
            S_WRITE_ATIME: begin
                if (!busy && !enable) begin
                    // Set gain to 16x (0x02 to register 0x0F)
                    register_address    <= 8'h8F;       // CONTROL reg (0x0F + 0x80)
                    mosi_data           <= 8'h02;       // AGAIN = 16x
                    read_write          <= 0;
                    enable              <= 1;
                    delay_counter       <= 24'd28_800;  // Wait integration time
                    state               <= S_WRITE_GAIN;
                end
            end
    
            S_WRITE_GAIN: begin
                if (delay_counter > 0) begin
                    delay_counter       <= delay_counter - 1;
                end else if (!busy) begin
                    // Start reading sequence with Clear Low Byte
                    register_address    <= 8'h94;       // CDATAL reg (0x14 | 0x80)
                    read_write          <= 1;
                    enable              <= 1;
                    state               <= S_READ_CLEAR_L;
                end
            end
            
            S_READ_CLEAR_L: begin
                if (!busy && !enable) begin
                    clear_l <= miso_data;
                    // Read Clear High Byte
                    register_address    <= 8'h95;       // CDATAH reg (0x15 | 0x80)
                    read_write          <= 1;
                    enable              <= 1;
                    state               <= S_READ_CLEAR_H;
                end
            end
            
            S_READ_CLEAR_H: begin
                if (!busy && !enable) begin
                    clear_h <= miso_data;
                    // Read Red Low Byte
                    register_address    <= 8'h96;       // RDATAL reg (0x16 | 0x80)
                    read_write          <= 1;
                    enable              <= 1;
                    state               <= S_READ_RED_L;
                end
            end
            
            S_READ_RED_L: begin
                if (!busy && !enable) begin
                    red_l               <= miso_data;
                    // Read Red High Byte
                    register_address    <= 8'h97;       // RDATAH reg (0x17 | 0x80)
                    read_write          <= 1;
                    enable              <= 1;
                    state               <= S_READ_RED_H;
                end
            end
            
            S_READ_RED_H: begin
                if (!busy && !enable) begin
                    red_h               <= miso_data;
                    // Read Green Low Byte
                    register_address    <= 8'h98;       // GDATAL reg (0x18 | 0x80)
                    read_write          <= 1;
                    enable              <= 1;
                    state               <= S_READ_GREEN_L;
                end
            end
            
            S_READ_GREEN_L: begin
                if (!busy && !enable) begin
                    green_l             <= miso_data;
                    // Read Green High Byte
                    register_address    <= 8'h99; // GDATAH reg (0x19 | 0x80)
                    read_write          <= 1;
                    enable              <= 1;
                    state               <= S_READ_GREEN_H;
                end
            end
            
            S_READ_GREEN_H: begin
                if (!busy && !enable) begin
                    green_h             <= miso_data;
                    // Read Blue Low Byte
                    register_address    <= 8'h9A; // BDATAL reg (0x1A | 0x80)
                    read_write          <= 1;
                    enable              <= 1;
                    state               <= S_READ_BLUE_L;
                end
            end
            
            S_READ_BLUE_L: begin
                if (!busy && !enable) begin
                    blue_l <= miso_data;
                    // Read Blue High Byte
                    register_address    <= 8'h9B; // BDATAH reg (0x1B | 0x80)
                    read_write          <= 1;
                    enable              <= 1;
                    state               <= S_READ_BLUE_H;
                end
            end
            
            S_READ_BLUE_H: begin
                if (!busy && !enable) begin
                    blue_h              <= miso_data;
                    state               <= S_UPDATE_OUTPUT;
                end
            end
            
            S_UPDATE_OUTPUT: begin
                // Combine low and high bytes
                clear                   <= {clear_h,    clear_l };
                red                     <= {red_h,      red_l   };
                green                   <= {green_h,    green_l };
                blue                    <= {blue_h,     blue_l  };
                data_valid              <= 1;               // Pulse valid signal
                state                   <= S_DONE;
            end
            
            S_DONE: begin
                // Prepare for next reading cycle
                delay_counter           <= 24'd28_800; // 2.4ms delay
                state                   <= S_WRITE_GAIN;
            end
        endcase
    end
end
endmodule