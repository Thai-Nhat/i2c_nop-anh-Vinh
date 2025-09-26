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
// Author:  RTL DN TEAM                                                                                  //
//                                                                                                         //
// Create Date:    26/09/2025                                                                              //
// Module Name:    I2C_INTERFACE                                                                              //
// Revision History:                                                                                       //
// Revision 0.01 - File Created (Author)                                                                   //
// Revision 0.02 - modified to work with Cyclone10 board                                                   //
// Property of Veron Group                                                                                 //
////////////////////////////////////////////////////////////////////////////////////////////////////////////

`timescale 1ns / 1ps
module I2C_INTERFACE#(
    parameter integer unsigned NUMBER_OF_DATA_BYTES         = 1,
    parameter integer unsigned NUMBER_OF_REGISTER_BYTES     = 1,
    parameter integer unsigned ADDRESS_WIDTH                = 7,
    parameter integer unsigned CHECK_FOR_CLOCK_STRETCHING   = 1,    //set to non zero value to enable
    parameter integer unsigned CLOCK_STRETCHING_MAX_COUNT   = 'hFF  //set to 0 to disable, max number of divider ticks to wait during stretch check
)(
    input   wire                                        clock,
    input   wire                                        reset_n,
    input   wire                                        enable,
    input   wire                                        read_write,
    input   wire    [(NUMBER_OF_DATA_BYTES*8)-1:0]      mosi_data,
    input   wire    [(NUMBER_OF_REGISTER_BYTES*8)-1:0]  register_address,
    input   wire    [ADDRESS_WIDTH-1:0]                 device_address,
    input   wire                                        pll_clk_300k,   // ADD: 300 kHz clock from PLL
    input   wire                                        pll_locked,     // ADD (optional)


    output  reg     [(NUMBER_OF_DATA_BYTES*8)-1:0]      miso_data,
    output  logic                                       busy,

    inout                                               external_serial_data,
    inout                                               external_serial_clock
);

localparam DATA_WIDTH                   = (NUMBER_OF_DATA_BYTES         != 0)   ? (NUMBER_OF_DATA_BYTES     * 8) : 8;
localparam REGISTER_WIDTH               = (NUMBER_OF_REGISTER_BYTES     != 0)   ? (NUMBER_OF_REGISTER_BYTES * 8) : 8;
localparam MAX_NUMBER_BYTES             = (DATA_WIDTH > REGISTER_WIDTH)         ? (DATA_WIDTH/8) : (REGISTER_WIDTH/8);
localparam CLOCK_STRETCHING_TIMER_WIDTH = (CLOCK_STRETCHING_MAX_COUNT   != 0)   ? $clog2(CLOCK_STRETCHING_MAX_COUNT) : 1;

wire            timeout_cycle_timer_clock;
wire            timeout_cycle_timer_reset_n;
wire            timeout_cycle_timer_enable;
logic           timeout_cycle_timer_load_count;
wire  [15:0]    timeout_cycle_timer_count;
wire            timeout_cycle_timer_expired;

wire divider_tick = divider_tick_r;
wire safe_reset_n = reset_n & pll_locked;  // use this if you have 'pll_locked'

typedef enum
{
    S_IDLE                  = 0,
    S_START                 = 1,
    S_WRITE_ADDR_W          = 2,
    S_CHECK_ACK             = 3,
    S_WRITE_REG_ADDR        = 4,
    S_RESTART               = 5,
    S_WRITE_ADDR_R          = 6,
    S_READ_REG              = 7,
    S_SEND_NACK             = 8,
    S_SEND_STOP             = 9,
    S_WRITE_REG_DATA        = 10,
    S_SEND_ACK              = 11
} state_type;

state_type                                  state;
state_type                                  _state;
state_type                                  post_state;
state_type                                  _post_state;
reg                                         serial_clock;
logic                                       _serial_clock;
reg     [ADDRESS_WIDTH:0]                   saved_device_address;
logic   [ADDRESS_WIDTH:0]                   _saved_device_address;
reg     [REGISTER_WIDTH-1:0]                saved_register_address;
logic   [REGISTER_WIDTH-1:0]                _saved_register_address;
reg     [DATA_WIDTH-1:0]                    saved_mosi_data;
logic   [DATA_WIDTH-1:0]                    _saved_mosi_data;
reg     [1:0]                               process_counter;
logic   [1:0]                               _process_counter;
reg     [3:0]                               bit_counter;
logic   [3:0]                               _bit_counter;
reg                                         serial_data;
logic                                       _serial_data;
reg                                         post_serial_data;
logic                                       _post_serial_data;
reg                                         last_acknowledge;
logic                                       _last_acknowledge;
logic                                       _saved_read_write;
reg                                         saved_read_write;
reg     [15:0]                              divider_counter;
logic   [15:0]                              _divider_counter;
logic   [DATA_WIDTH-1:0]                    _miso_data;
logic                                       _busy;
logic                                       serial_data_output_enable;
logic                                       serial_clock_output_enable;
logic   [$clog2(MAX_NUMBER_BYTES)-1:0]      _byte_counter;
reg     [$clog2(MAX_NUMBER_BYTES)-1:0]      byte_counter;

reg     [CLOCK_STRETCHING_TIMER_WIDTH-1:0] counter;
logic   [CLOCK_STRETCHING_TIMER_WIDTH-1:0] _counter;


assign external_serial_clock        = (serial_clock_output_enable)  ? serial_clock : 1'bz;
assign external_serial_data         = (serial_data_output_enable)   ? serial_data  : 1'bz;

assign timeout_cycle_timer_clock    = clock;
assign timeout_cycle_timer_reset_n  = reset_n;
assign timeout_cycle_timer_enable   = (CLOCK_STRETCHING_MAX_COUNT != 0) ? divider_tick : 0;
assign timeout_cycle_timer_count    = CLOCK_STRETCHING_MAX_COUNT;


// ==================== PLL-to-system clock synchronizer ====================
// Goal: sync pll_clk_300k to the 'clock' domain (the FSM system clock)
reg [2:0] pll_sync;
always @(posedge clock or negedge safe_reset_n) begin
    if (!safe_reset_n) begin
        pll_sync <= 3'b111;      // assume idle level is 1
    end else begin
        pll_sync <= {pll_sync[1:0], pll_clk_300k};
    end
end

// ==================== Falling-edge detect: make divider_tick ==============
// divider_tick = 1 for one 'clock' cycle when pll_clk_300k has a falling edge
reg divider_tick_r;
always @(posedge clock or negedge safe_reset_n) begin
    if (!safe_reset_n) begin
        divider_tick_r <= 1'b0;
    end else begin
        // falling edge (1 -> 0) after synchronization
        divider_tick_r <= (pll_sync[2:1] == 2'b10);
    end
end




always_comb begin
    _state                          = state;
    _post_state                     = post_state;
    _process_counter                = process_counter;
    _bit_counter                    = bit_counter;
    _last_acknowledge               = last_acknowledge;
    _miso_data                      = miso_data;
    _saved_read_write               = saved_read_write;
    _divider_counter                = divider_counter;
    _saved_register_address         = saved_register_address;
    _saved_device_address           = saved_device_address;
    _saved_mosi_data                = saved_mosi_data;
    _serial_data                    = serial_data;
    _serial_clock                   = serial_clock;
    _post_serial_data               = post_serial_data;
    _byte_counter                   = byte_counter;
    timeout_cycle_timer_load_count  = 0;
    serial_data_output_enable       = 1;
    busy                            = (state == S_IDLE) ? 0 : 1;

    if (state != S_IDLE && process_counter != 1 && process_counter != 2) begin
        serial_clock_output_enable   = 1;
    end
    else begin
        serial_clock_output_enable   = 0;
    end

    if (process_counter == 0) begin
        timeout_cycle_timer_load_count  = 1;
    end

    case (state)
        S_IDLE: begin
            serial_data_output_enable       = 0;
            _process_counter                = 0;
            _bit_counter                    = 0;
            _last_acknowledge               = 0;
            _serial_data                    = 1;
            _serial_clock                   = 1;
            _saved_read_write               = read_write;

            if (NUMBER_OF_DATA_BYTES == 0) begin
                _saved_mosi_data        = '0;
            end
            else begin
                _saved_mosi_data        = mosi_data;
            end
            if (NUMBER_OF_REGISTER_BYTES == 0) begin
                _saved_register_address = '0;
            end
            else begin
                _saved_register_address = register_address;
            end
            if (ADDRESS_WIDTH == 0) begin
                _saved_device_address   = '0;
            end
            else begin
                _saved_device_address   = {device_address, 1'b0};  // write
            end

            if (enable) begin
                _state                  = S_START;
                _post_state             = S_WRITE_ADDR_W;
            end
        end

        S_START: begin
            if (divider_tick) begin
                case (process_counter)
                    0: begin
                        _process_counter    = 1;
                    end
                    1: begin
                        _serial_data        = 0;
                        _process_counter    = 2;
                    end
                    2:  begin
                        _bit_counter        = ADDRESS_WIDTH + 1;
                        _process_counter    = 3;
                    end
                    3:  begin
                        _serial_clock       = 0;
                        _process_counter    = 0;
                        _state              = post_state;
                        _serial_data        = saved_device_address[ADDRESS_WIDTH];
                    end
                endcase
            end
        end
        S_WRITE_ADDR_W: begin
            if (process_counter == 3 && bit_counter == 0) begin
                serial_data_output_enable   = 0;
            end
            if (divider_tick) begin
                case (process_counter)
                    0: begin
                        _serial_clock       = 1;
                        _process_counter    = 1;
                    end
                    1: begin
                        //check for clock stretching
                        if (CLOCK_STRETCHING_MAX_COUNT != 0) begin
                            if (timeout_cycle_timer_expired) begin
                                _process_counter    = 0;
                                _state              = S_IDLE;
                            end
                        end
                        if (external_serial_clock || !CHECK_FOR_CLOCK_STRETCHING) begin
                            _process_counter    = 2;
                            _state              = S_WRITE_ADDR_W;
                        end
                    end
                    2: begin
                        _serial_clock       = 0;
                        _bit_counter        = bit_counter -   1;
                        _process_counter    = 3;
                    end
                    3: begin
                        _process_counter    = 0;

                        if (bit_counter == 0) begin
                            _post_serial_data       = saved_register_address[REGISTER_WIDTH-1];
                            _saved_register_address = {saved_register_address[REGISTER_WIDTH-2:0], saved_register_address[REGISTER_WIDTH-1]};
                            _post_state             = S_WRITE_REG_ADDR;
                            _state                  = S_CHECK_ACK;
                            _bit_counter            = 8;
                            _byte_counter           = NUMBER_OF_REGISTER_BYTES - 1;
                        end
                        else begin
                            _serial_data            = saved_device_address[ADDRESS_WIDTH-1];
                        end

                        if (ADDRESS_WIDTH != 0) begin
                            _saved_device_address   = {saved_device_address[ADDRESS_WIDTH-1:0], saved_device_address[ADDRESS_WIDTH]};
                        end
                    end
                endcase
            end
        end

        S_CHECK_ACK: begin
            serial_data_output_enable   = 0;
            if (divider_tick) begin
                case (process_counter)
                    0: begin
                        _serial_clock       = 1;
                        _process_counter    = 1;
                    end
                    1: begin
                        if (CLOCK_STRETCHING_MAX_COUNT != 0) begin
                            if (timeout_cycle_timer_expired) begin
                                _process_counter    = 0;
                                _state              = S_IDLE;
                            end
                        end
                        //check for clock stretching
                        if (external_serial_clock || !CHECK_FOR_CLOCK_STRETCHING) begin
                            _last_acknowledge   = 0;
                            _process_counter    = 2;
                        end
                    end
                    2: begin
                        _serial_clock   = 0;

                        if (external_serial_data == 0) begin
                            _last_acknowledge   = 1;    
                        end
                        _process_counter    = 3;
                    end
                    3:  begin
                        if (last_acknowledge == 1) begin
                            _last_acknowledge   = 0;
                            _serial_data        = post_serial_data;
                            _state              = post_state;
                        end
                        else begin
                            _state  = S_SEND_STOP;
                        end
                        _process_counter = 0;
                    end
                endcase
            end
        end

        S_WRITE_REG_ADDR: begin
            if (process_counter == 3 && bit_counter == 0) begin
                serial_data_output_enable   = 0;
            end
            if (divider_tick) begin
                case (process_counter)
                    0: begin
                        _serial_clock       = 1;
                        _process_counter    = 1;
                    end
                    1: begin
                        if (CLOCK_STRETCHING_MAX_COUNT != 0) begin
                            if (timeout_cycle_timer_expired) begin
                                _process_counter    = 0;
                                _state              = S_IDLE;
                            end
                        end
                        //check for clock stretching
                        if (external_serial_clock || !CHECK_FOR_CLOCK_STRETCHING) begin
                            _last_acknowledge   = 0;
                            _process_counter    = 2;
                        end
                    end
                    2: begin
                        _serial_clock       = 0;
                        _bit_counter        = bit_counter - 1;
                        _process_counter    = 3;
                    end
                    3: begin
                        if (bit_counter == 0) begin
                            _byte_counter   = byte_counter - 1;
                            _bit_counter    = 8;
                            _serial_data    = 0;
                            _state          = S_CHECK_ACK;

                            if (byte_counter == 0) begin
                                if (saved_read_write == 0) begin
                                    _post_state         = S_WRITE_REG_DATA;
                                    _post_serial_data   = saved_mosi_data[DATA_WIDTH-1];
                                    _saved_mosi_data    = {saved_mosi_data[DATA_WIDTH-2:0], saved_mosi_data[DATA_WIDTH-1]};
                                    _byte_counter       = NUMBER_OF_DATA_BYTES - 1;
                                end
                                else begin
                                    _post_state         = S_RESTART;
                                    _byte_counter       = 0;
                                    _post_serial_data   = 1;
                                end
                            end
                            else begin
                                _post_state = S_WRITE_REG_ADDR;
                            end
                        end
                        else begin
                            _serial_data            = saved_register_address[REGISTER_WIDTH-1];
                            _saved_register_address = {saved_register_address[REGISTER_WIDTH-2:0], saved_register_address[REGISTER_WIDTH-1]}; 
                        end
                        _process_counter    = 0;
                    end
                endcase
            end
        end

        S_WRITE_REG_DATA: begin
            if (process_counter == 3 && bit_counter == 0) begin
                serial_data_output_enable   = 0;
            end

            if (divider_tick) begin
                case (process_counter)
                    0: begin
                        _serial_clock       = 1;
                        _process_counter    = 1;
                    end
                    1: begin
                        if (CLOCK_STRETCHING_MAX_COUNT != 0) begin
                            if (timeout_cycle_timer_expired) begin
                                _process_counter    = 0;
                                _state              = S_IDLE;
                            end
                        end
                        //check for clock stretching
                        if (external_serial_clock || !CHECK_FOR_CLOCK_STRETCHING) begin
                            _last_acknowledge   = 0;
                            _process_counter    = 2;
                        end
                    end
                    2: begin
                        _serial_clock       = 0;
                        _bit_counter        = bit_counter - 1;
                        _process_counter    = 3;
                    end
                    3: begin
                        if (bit_counter == 0) begin
                            _byte_counter       = byte_counter - 1;
                            _state              = S_CHECK_ACK;
                            _bit_counter        = 8;
                            _serial_data        = 0;

                            if (byte_counter == 0) begin
                                _byte_counter       = 0;
                                _post_state         = S_SEND_STOP;
                                _post_serial_data   = 0;
                            end
                            else begin
                                _post_state         = S_WRITE_REG_DATA;
                                _post_serial_data   = saved_mosi_data[DATA_WIDTH-1];
                                _saved_mosi_data    = {saved_mosi_data[DATA_WIDTH-2:0], saved_mosi_data[DATA_WIDTH-1]};
                            end

                        end
                        else begin
                            _serial_data        = saved_mosi_data[DATA_WIDTH-1];
                            _saved_mosi_data    = {saved_mosi_data[DATA_WIDTH-2:0], saved_mosi_data[DATA_WIDTH-1]};
                        end
                        _process_counter        = 0;
                    end
                endcase
            end
        end

        S_RESTART: begin
            if (divider_tick) begin
                case (process_counter)
                    0: begin
                        _process_counter    = 1;
                    end
                    1: begin
                        _process_counter    = 2;
                        _serial_clock       = 1;
                    end
                    2: begin
                        _process_counter    = 3;
                    end
                    3: begin
                        _state                      = S_START;
                        _post_state                 = S_WRITE_ADDR_R;
                        _process_counter            = 0;

                        if (ADDRESS_WIDTH == 0) begin
                            _saved_device_address   = '1;
                        end
                        else begin
                            _saved_device_address[0]    = 1; //read
                        end
                    end
                endcase
            end
        end

        S_WRITE_ADDR_R: begin
            if (process_counter == 3 && bit_counter == 0) begin
                serial_data_output_enable   = 0;
            end
            
            if (divider_tick) begin
                case (process_counter)
                    0: begin
                        _serial_clock       = 1;
                        _process_counter    = 1;
                    end
                    1: begin
                        if (CLOCK_STRETCHING_MAX_COUNT != 0) begin
                            if (timeout_cycle_timer_expired) begin
                                _process_counter    = 0;
                                _state              = S_IDLE;
                            end
                        end
                        //check for clock stretching
                        if (external_serial_clock || !CHECK_FOR_CLOCK_STRETCHING) begin
                            _last_acknowledge   = 0;
                            _process_counter    = 2;
                        end
                    end
                    2: begin
                        _serial_clock       = 0;
                        _bit_counter        = bit_counter - 1;
                        _process_counter    = 3;
                    end
                    3: begin
                        _process_counter    = 0;

                        if (bit_counter == 0) begin
                            _post_state         = S_READ_REG;
                            _post_serial_data   = 0;
                            _state              = S_CHECK_ACK;
                            _bit_counter        = 8;
                            _byte_counter       = NUMBER_OF_DATA_BYTES - 1;
                        end
                        else begin
                            _serial_data            = saved_device_address[ADDRESS_WIDTH-1];
                        end

                        if (ADDRESS_WIDTH != 0) begin
                            _saved_device_address   = {saved_device_address[ADDRESS_WIDTH-1:0], saved_device_address[ADDRESS_WIDTH]};
                        end
                    end
                endcase
            end
        end

        S_READ_REG: begin
            if (process_counter != 3) begin
                serial_data_output_enable   = 0;
            end

            if (divider_tick) begin
                case (process_counter)
                    0: begin
                        _serial_clock       = 1;
                        _process_counter    = 1;
                    end
                    1: begin
                        if (CLOCK_STRETCHING_MAX_COUNT != 0) begin
                            if (timeout_cycle_timer_expired) begin
                                _process_counter    = 0;
                                _state              = S_IDLE;
                            end
                        end
                        //check for clock stretching
                        if (external_serial_clock || !CHECK_FOR_CLOCK_STRETCHING) begin
                            _last_acknowledge   = 0;
                            _process_counter    = 2;
                        end
                    end
                    2: begin
                        _serial_clock               = 0;
                        //sample data on this rising edge of scl
                        _miso_data[0]               = external_serial_data;
                        _miso_data[DATA_WIDTH-1:1]  = miso_data[DATA_WIDTH-2:0];
                        _bit_counter                = bit_counter - 1;
                        _process_counter            = 3;
                    end
                    3: begin
                        if (bit_counter == 0) begin
                            _byte_counter   = byte_counter - 1;
                            _bit_counter    = 8;
                            _serial_data    = 0;

                            if (byte_counter == 0) begin
                                _byte_counter   = 0;
                                _state          = S_SEND_NACK;
                            end
                            else begin
                                _post_state     = S_READ_REG;
                                _state          = S_SEND_ACK;
                            end
                        end
                        _process_counter    = 0;
                    end
                endcase
            end
        end

        S_SEND_NACK: begin
            if (divider_tick) begin
                case (process_counter)
                    0: begin
                        _serial_clock       = 1;
                        _serial_data        = 1;
                        _process_counter    = 1;
                    end
                    1: begin
                        if (CLOCK_STRETCHING_MAX_COUNT != 0) begin
                            if (timeout_cycle_timer_expired) begin
                                _process_counter    = 0;
                                _state              = S_IDLE;
                            end
                        end
                        //check for clock stretching
                        if (external_serial_clock || !CHECK_FOR_CLOCK_STRETCHING) begin
                            _last_acknowledge   = 0;
                            _process_counter    = 2;
                        end
                    end
                    2: begin
                        _process_counter    = 3;
                        _serial_clock       = 0;
                    end
                    3: begin
                        _state              = S_SEND_STOP;
                        _process_counter    = 0;
                        _serial_data        = 0;
                    end
                endcase
            end
        end

        S_SEND_ACK: begin
            if (divider_tick) begin
                case (process_counter)
                    0: begin
                        _serial_clock       = 1;
                        _process_counter    = 1;
                        _serial_data        = 0;
                    end
                    1: begin
                        if (CLOCK_STRETCHING_MAX_COUNT != 0) begin
                            if (timeout_cycle_timer_expired) begin
                                _process_counter    = 0;
                                _state              = S_IDLE;
                            end
                        end
                        //check for clock stretching
                        if (external_serial_clock || !CHECK_FOR_CLOCK_STRETCHING) begin
                            _last_acknowledge   = 0;
                            _process_counter    = 2;
                        end
                    end
                    2: begin
                        _process_counter    = 3;
                        _serial_clock       = 0;
                    end
                    3: begin
                        _state              = post_state;
                        _process_counter    = 0;
                    end
                endcase
            end
        end
        
        S_SEND_STOP: begin
            if (divider_tick) begin
                case (process_counter)
                    0: begin
                        _serial_clock       = 1;
                        _process_counter    = 1;
                    end
                    1: begin
                        if (CLOCK_STRETCHING_MAX_COUNT != 0) begin
                            if (timeout_cycle_timer_expired) begin
                                _process_counter    = 0;
                                _state              = S_IDLE;
                            end
                        end
                        //check for clock stretching
                        if (external_serial_clock || !CHECK_FOR_CLOCK_STRETCHING) begin
                            _last_acknowledge   = 0;
                            _process_counter    = 2;
                        end
                    end
                    2: begin
                        _process_counter    = 3;
                        _serial_data        = 1;
                    end
                    3: begin
                        _state  = S_IDLE;
                    end
                endcase
            end
        end
    endcase
end

always_ff @(posedge clock) begin
    if (!reset_n) begin
        state                   <= S_IDLE;
        post_state              <= S_IDLE;
        process_counter         <= 0;
        bit_counter             <= 0;
        last_acknowledge        <= 0;
        miso_data               <= 0;
        saved_read_write        <= 0;
        divider_counter         <= 0;
        saved_device_address    <= 0;
        saved_register_address  <= 0;
        saved_mosi_data         <= 0;
        serial_clock            <= 0;
        serial_data             <= 0;
        post_serial_data        <= 0;
        byte_counter            <= 0;
    end
    else begin
        state                   <= _state;
        post_state              <= _post_state;
        process_counter         <= _process_counter;
        bit_counter             <= _bit_counter;
        last_acknowledge        <= _last_acknowledge;
        miso_data               <= _miso_data;
        saved_read_write        <= _saved_read_write;
        divider_counter         <= _divider_counter;
        saved_device_address    <= _saved_device_address;
        saved_register_address  <= _saved_register_address;
        saved_mosi_data         <= _saved_mosi_data;
        serial_clock            <= _serial_clock;
        serial_data             <= _serial_data;
        post_serial_data        <= _post_serial_data;
        byte_counter            <= _byte_counter;
    end
 end

 
always_comb begin
    _counter    =   counter;

    if (counter == 0) begin
        timeout_cycle_timer_expired = 1;
    end
    else begin
        timeout_cycle_timer_expired = 0;
    end

    if (timeout_cycle_timer_enable) begin
        if (timeout_cycle_timer_load_count) begin
            _counter = timeout_cycle_timer_count;
        end
        else begin
            if (counter == 0) begin
            end
            else begin
                _counter    = counter - 1;
            end
        end
    end
end

always_ff @(posedge clock or negedge reset_n) begin
    if (!reset_n) begin
        counter <=  '1;
    end
    else begin
        counter <=  _counter;
    end
end
    
endmodule