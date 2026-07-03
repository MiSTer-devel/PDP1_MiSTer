//============================================================================
//  PDP1 emulator for MiSTer
//  Copyright (c) 2018 Hrvoje Cavrak
//  Based on Defender by Sorgelig (c) 2017
//
//  This program is free software; you can redistribute it and/or modify it
//  under the terms of the GNU General Public License as published by the Free
//  Software Foundation; either version 2 of the License, or (at your option)
//  any later version.
//
//  This program is distributed in the hope that it will be useful, but WITHOUT
//  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
//  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
//  more details.
//
//  You should have received a copy of the GNU General Public License along
//  with this program; if not, write to the Free Software Foundation, Inc.,
//  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//============================================================================

module emu
(
   `include "sys/emu_ports.vh"
);

///////// Default values for ports not used in this core /////////

assign ADC_BUS  = 'Z;
assign USER_OUT = '1;
assign {UART_RTS, UART_TXD, UART_DTR} = 0;
assign {SD_SCK, SD_MOSI, SD_CS} = 'Z;
assign {SDRAM_DQ, SDRAM_A, SDRAM_BA, SDRAM_CLK, SDRAM_CKE, SDRAM_DQML, SDRAM_DQMH, SDRAM_nWE, SDRAM_nCAS, SDRAM_nRAS, SDRAM_nCS} = 'Z;
assign {DDRAM_CLK, DDRAM_BURSTCNT, DDRAM_ADDR, DDRAM_DIN, DDRAM_BE, DDRAM_RD, DDRAM_WE} = '0;

`ifdef MISTER_DUAL_SDRAM
assign {SDRAM2_DQ, SDRAM2_A, SDRAM2_BA, SDRAM2_CLK, SDRAM2_nWE, SDRAM2_nCAS, SDRAM2_nRAS, SDRAM2_nCS} = 'Z;
`endif

`ifdef MISTER_FB
assign FB_EN = 0;
assign FB_FORMAT = 0;
assign FB_WIDTH = 0;
assign FB_HEIGHT = 0;
assign FB_BASE = 0;
assign FB_STRIDE = 0;
assign FB_FORCE_BLANK = 0;
`ifdef MISTER_FB_PALETTE
assign FB_PAL_CLK = 0;
assign FB_PAL_ADDR = 0;
assign FB_PAL_DOUT = 0;
assign FB_PAL_WR = 0;
`endif
`endif

assign VGA_SL = 0;
assign VGA_F1 = 0;
assign VGA_SCALER = 0;
assign VGA_DISABLE = 0;
assign HDMI_FREEZE = 0;
assign HDMI_BLACKOUT = 0;
assign HDMI_BOB_DEINT = 0;

assign AUDIO_S = 0;
assign AUDIO_L = 0;
assign AUDIO_R = 0;
assign AUDIO_MIX = 0;

assign LED_POWER = 0;
assign LED_DISK = 0;
assign BUTTONS = 0;

//////////////////////////////////////////////////////////////////

`include "build_id.v" 
localparam CONF_STR = {
   "PDP1;;",
   "-;",
   "F1,PDPRIMBIN,Load PDP/RIM/BIN;",
   "T[5],Enable RIM mode;",
   "T[6],Disable RIM mode;", 
   "-;",
   "R[7],Reset;",
   "T[0],Reset;",
   "R[0],Reset and close OSD;",
   "-;",
   "O[1],Aspect Ratio,Original,Wide;",
   "O[4],Hardware multiply,No,Yes;",
   "-;",
   "O[8],Var. brightness,Yes,No;",  
   "O[9],CRT wait,No,Yes;",
   "-;", 
   "J,Left,Right,Thrust,Fire,HyperSpace;",
   "v,1;",
   "V,v",`BUILD_DATE
};

////////////////////   CLOCKS   ///////////////////

pll pll (
   .refclk(CLK_50M),
   .rst(1'b0),
   .outclk_0(clk_108),
   .locked(pll_locked)
);

////////////////////  WIRES  //////////////////////

wire [127:0] status;                                                  /* Signal carries menu settings */
wire  [31:0] BUS_out;                                                 /* CPU signals to console */
wire  [1:0] current_output_device;                                   /* Currently selected output device */

wire        clk_108, pll_locked;                                     /* Clock wires */
wire        clk_sys = CLK_50M;                                       /* PDP1 logic and HPS bridge clock */
wire        kbd_read_strobe, console_switch_strobe;                  /* These signal when a key was pressed */

wire        ioctl_download, ioctl_wr, send_next_tape_char;           /* Tape (file download) ioctl interface */
wire [26:0] ioctl_addr;
wire [15:0] ioctl_index;
wire  [7:0] ioctl_dout;

wire [10:0] ps2_key, console_switches;                               /* Pressed key on keyboard, pressed switches on console */
wire  [5:0] sense_switches;

wire [31:0] joystick_0, joystick_1;                                  /* Pressed keys on joysticks */
wire  [1:0] buttons;                                                 /* OSD/user buttons from hps_io */
wire        forced_scandoubler;

wire  [6:0] char_output_w, kbd_char_out;                             /* Pressed keys to typewriter VGA out */
wire        char_strobe_w, halt_w, key_was_processed_w;              /* Strobe / ACK signalling of pressed keys */
                        
wire [17:0] test_word, test_address, AC_out, IO_out, DI_out;         /* Provide signals for console */
wire [11:0] PC_out, AB_out;

wire  [7:0] r_crt, g_crt, b_crt,                                     /* Per-device RGB signals, for CRT, Teletype and Console emulation */
            r_tty, g_tty, b_tty, 
            r_con, g_con, b_con;

wire  [5:0] selected_ptr_x;                                          /* Cursor coordinates for toggling console test switches */
wire  [4:0] selected_ptr_y;

wire        hw_mul_enabled = `menu_hardware_multiply;                /* Is hardware multiplication enabled from menu ? */

wire [17:0] data_out;                                                /* Data bus, CPU to RAM */
wire        ram_write_enable;                                        /* When set, writes to main RAM memory */

wire [9:0]  pixel_x_addr_wire, pixel_y_addr_wire;                    /* Lines connecting the CPU and Type 30 CRT */
wire [2:0]  pixel_brightness_wire;
wire        pixel_shift_wire;

wire [7:0]  joystick_emu;                                            /* Output from keyboard module to feed in as spacewar controls */

//////////////////  REGISTERS  ////////////////////

reg        old_state, old_download, old_key_was_processed;           /* Used to detect a rising edge */
reg        ioctl_wait = 0;                                           /* When 1, signal through HPS bus that we are not ready to receive more data */

reg        current_case;                                             /* 0 = lowercase currently active, 1 = uppercase active */
reg  [1:0] current_output;                                           /* What device video output is active? Can be CRT, Typewriter or Console */
reg  [0:0] cpu_running = 1'b1;                                       /* If set to 0, cpu is paused */

reg [11:0] write_address = 12'd0;                                    /* Addresses for writing to memory and start jump location after loading a program in RIM mode or RESET */
reg [11:0] start_address = 12'd4;

reg [17:0] tape_rcv_word,                                            /* tape_rcv_word used to store received binary word from tape */
           io_word;                                                  /* io_word used to provide spacewar gamepad controls */
           
reg        write_enable, rim_mode_enabled;                           /* Enables writing to memory or activating the read in mode (i.e. something like a paper tape bootloader) */

reg [35:0] tape_read_buffer = 36'b0;                                 /* Buffer for storing lines received from paper tape */
reg [31:0] timeout = 0;                                              /* Timeout provides a control mechanism to abort a "stuck" paper tape download */
reg [10:0] horizontal_counter, vertical_counter;                     /* Position counters used for generating the video signal, common to all three video output modules */


/////////////////  ASSIGNMENTS  ///////////////////

assign VIDEO_ARX = `menu_aspect_ratio ? 13'd16 : 13'd5;
assign VIDEO_ARY = `menu_aspect_ratio ? 13'd9  : 13'd4;

assign LED_USER = key_was_processed_w;

assign CLK_VIDEO = clk_108;
assign CE_PIXEL  = 1'b1;

wire reset = RESET | status[0] | buttons[1] | `menu_reset;

/* Convert joystick / keyboard commands into PDP1 spacewar IO register 18-bit word */
assign io_word = {joystick_0[1] | joystick_emu[1] | joystick_0[`joystick_left]   | joystick_0[`joystick_hyperspace],       /* Hyperspace is triggered when both left */
                  joystick_0[0] | joystick_emu[3] | joystick_0[`joystick_right]  | joystick_0[`joystick_hyperspace],       /* and right are pressed simultaneously.  */
                  joystick_0[2] | joystick_emu[2] | joystick_0[`joystick_thrust], 
                  joystick_0[3] | joystick_emu[0] | joystick_0[`joystick_fire],
                      
                  {10{1'b0}}, 
                      
                  joystick_1[1] | joystick_emu[5] | joystick_1[`joystick_left]   | joystick_1[`joystick_hyperspace], 
                  joystick_1[0] | joystick_emu[7] | joystick_1[`joystick_right]  | joystick_1[`joystick_hyperspace],
                  joystick_1[2] | joystick_emu[6] | joystick_1[`joystick_thrust],  
                  joystick_1[3] | joystick_emu[4] | joystick_1[`joystick_fire]
               };
               
///////////////////  MODULES  /////////////////////


hps_io #(.CONF_STR(CONF_STR), .PS2DIV(200)) hps_io
(
   .clk_sys(clk_sys),
   .HPS_BUS(HPS_BUS),
   .EXT_BUS(),
   .gamma_bus(),

   .buttons(buttons),
   .status(status),
   .status_in(128'd0),
   .status_set(1'b0),
   .status_menumask(16'd0),
   .forced_scandoubler(forced_scandoubler),
   .video_rotated(1'b0),
   .new_vmode(1'b0),

   .ps2_kbd_led_status(3'd0),
   .ps2_kbd_led_use(3'd0),

   .ioctl_download(ioctl_download),
   .ioctl_wr(ioctl_wr),
   .ioctl_addr(ioctl_addr),
   .ioctl_dout(ioctl_dout),

   .ioctl_index(ioctl_index),
   .ioctl_upload_req(1'b0),
   .ioctl_upload_index(8'd0),
   .ioctl_din(8'd0),
   .ioctl_wait(ioctl_wait),

   .info_req(1'b0),
   .info(8'd0),
         
   .joystick_0(joystick_0),
   .joystick_1(joystick_1),
   
   .ps2_key(ps2_key)
);

keyboard keyboard(
   .clk(clk_sys),
   .ps2_key(ps2_key),
   .key_was_processed(key_was_processed_w),
      
   .kbd_read_strobe(kbd_read_strobe),
   .console_switch_strobe(console_switch_strobe),
   .console_switches(console_switches),
   .kbd_char_out(kbd_char_out),
   .current_output_device(current_output_device),
   .joystick_emu(joystick_emu),
      
   .selected_ptr_x(selected_ptr_x),
   .selected_ptr_y(selected_ptr_y)
);


pdp1_vga_typewriter typewriter (
   .clk(clk_108), 
          
   .horizontal_counter(horizontal_counter),
   .vertical_counter(vertical_counter),

   .red_out(r_tty),
   .green_out(g_tty), 
   .blue_out(b_tty),
             
   .char_in_kbd(kbd_char_out),
   .char_in_pdp(char_output_w),
             
   .have_typewriter_data(char_strobe_w),
   .have_keyboard_data(kbd_read_strobe)
);


pdp1_vga_console console (
   .clk(clk_108), 
             
   .horizontal_counter(horizontal_counter),
   .vertical_counter(vertical_counter),
             
   .red_out(r_con),
   .green_out(g_con), 
   .blue_out(b_con),
             
   .selected_ptr_x(selected_ptr_x),
   .selected_ptr_y(selected_ptr_y),       
   .console_switch_strobe(console_switch_strobe),
             
   .sense_switches(sense_switches),
   .test_word(test_word),
   .test_address(test_address),
             
   .console_switches(console_switches),

   .PC_in(PC_out),
   .AC_in(AC_out),
   .IO_in(IO_out),
   .AB_in(AB_out),
   .DI_in(DI_out),
   .BUS_in(BUS_out),
   .RIM_in(rim_mode_enabled)
);


pdp1_main_ram ram_memory(
   .address_a(AB_out), 
   .clock_a(clk_sys), 
   .data_a(data_out),
   .wren_a(ram_write_enable), 
   .q_a(DI_out),
                         
   .clock_b(clk_sys),
   .address_b(write_address),
   .data_b(tape_rcv_word),
   .wren_b(write_enable)
);


pdp1_vga_crt type30_crt(
   .clk(clk_108), 
      
   .horizontal_counter(horizontal_counter),
   .vertical_counter(vertical_counter),
             
   .red_out(r_crt),
   .green_out(g_crt), 
   .blue_out(b_crt),
                   
   .pixel_available(pixel_shift_wire),
   
   .pixel_x_i(pixel_x_addr_wire),
   .pixel_y_i(pixel_y_addr_wire),
   .pixel_brightness(pixel_brightness_wire),
   .variable_brightness(~`menu_variable_brightness)
);


cpu pdp1_cpu(
   .clk(clk_sys), 
   .rst(reset | rim_mode_enabled), 
   .MEM_ADDR(AB_out), 
   .DI(DI_out), 
   .MEM_BUFF(data_out), 
   .WRITE_ENABLE(ram_write_enable),
      
   .PC(PC_out),
   .AC(AC_out),
   .IO(IO_out),
   .BUS_out(BUS_out),
                      
   .pixel_x_out(pixel_x_addr_wire),
   .pixel_y_out(pixel_y_addr_wire),
   .pixel_shift_out(pixel_shift_wire),
   .pixel_brightness(pixel_brightness_wire),
             
   .gamepad_in(io_word),
          
   .start_address(start_address),
             
   .typewriter_char_out(char_output_w),
   .typewriter_strobe_out(char_strobe_w),
             
   .typewriter_char_in(kbd_char_out),
   .typewriter_strobe_in(kbd_read_strobe),
   .typewriter_strobe_ack(key_was_processed_w),
             
   .send_next_tape_char(send_next_tape_char),
   .is_char_available(ioctl_wait),
   .tape_rcv_word(tape_rcv_word),
             
   .sense_switches(sense_switches),
   .test_word(test_word),
   .test_address(test_address),
             
   .halt(halt_w),
   .cpu_running(cpu_running & ~`power_switch),
          
   .console_switches(console_switches),
         
   .crt_wait(`menu_crt_wait),
   .hw_mul_enabled(hw_mul_enabled)            
);       


////////////////  ALWAYS BLOCKS  //////////////////

always @(posedge clk_sys) begin
   if (`continue_button || `start_button)
      cpu_running <= 1'b1;
   
   if (`stop_button)
      cpu_running <= 1'b0;      
end

/* Tape RIM loader and memory image loader */

always @(posedge clk_sys) begin
        reg [7:0] cnt;
        reg       old_send_next_tape_char;

        /* In RIM mode, these two instructions write code to memory (dio) and then jump to beginning to execute (jmp) */
        localparam jmp   = 6'o60,    
                   dio   = 6'o32;
        
        old_send_next_tape_char <= send_next_tape_char;
        old_download <= ioctl_download;
        write_enable <= 1'b0;

        if (`menu_enable_rim || `readin_button)  
            rim_mode_enabled <= 1'b1;
        
        else if (`menu_disable_rim) 
        begin
            rim_mode_enabled <= 1'b0;                                                     
            ioctl_wait <= 1'b0;
        end
        
        if(~old_download && ioctl_download) begin
                cnt <= 8'b0;
                timeout <= 32'b0;
        end         
        
        timeout <= timeout + 1'b1;
           
        /* 8th bit must be a one in binary mode. If not, don't even increment counter - simply ignore it */    
        if(ioctl_wr && ioctl_dout[7]) begin                 
            tape_read_buffer <= { tape_read_buffer[29:0], ioctl_dout[5:0] };         /* Shift in 6 bits */
            cnt <= cnt + 1'b1;
            timeout <= rim_mode_enabled ? 'b0 : timeout;
        end   
              
        
        /* Make one 18-bit word from every 3 bytes, enable write and raise ioctl_wait to skip read in next clock cycle */            

        if (! rim_mode_enabled) 
        begin
            if (cnt == 8'd3)  // We read enough data to form a word, ioctl_wait will be cleared when send_next_tape_char is pulsed
            begin
               ioctl_wait <= 1'b1;  // We received a character, don't receive anymore until the CPU reads it
               tape_rcv_word <= tape_read_buffer[17:0];
               tape_read_buffer <= 36'b0;       
               cnt <= 8'b0;               
            end

            /* Detect falling edge of send_next_tape_char input from CPU, that means we can proceed with receiving */          
            if (old_send_next_tape_char && ~send_next_tape_char) begin 
               ioctl_wait <= 0;                 
            end

        end
        
        /* RIM loader mode */
        else 
        begin          
            if (cnt == 8'd6)
            begin
               cnt <= 8'b0;               

               if (tape_read_buffer[35:30] == jmp) begin
                  start_address <= tape_read_buffer[29:18];
                  rim_mode_enabled <= 1'b0;                 
                  
                  tape_rcv_word <= tape_read_buffer[17:0];                                   
                  ioctl_wait <= 1'b1;  // We received a character, don't receive anymore until the CPU reads it                  
               end
               
               
               if (tape_read_buffer[35:30] == dio) begin             
                  write_address <= tape_read_buffer[29:18];
                  tape_rcv_word <= tape_read_buffer[17:0];
                  write_enable <= 1'b1;      
               end

               tape_read_buffer <= 36'b0;       
            end
         
        end
        
        /* Timeout after last successful read should not exceed 1 s */
        if(!ioctl_download || timeout > 32'd50000000) ioctl_wait <= 0;
end


/* Video generation */
         
always @(posedge clk_108) begin
   case (current_output_device)
      `output_crt:      {VGA_R, VGA_G, VGA_B} <= {r_crt, g_crt, b_crt};
      `output_console:  {VGA_R, VGA_G, VGA_B} <= {r_con, g_con, b_con};    
      `output_teletype: {VGA_R, VGA_G, VGA_B} <= {r_tty, g_tty, b_tty};          
   endcase  

   /* Common video routines for generating blanking and sync signals */

   VGA_HS <= ((horizontal_counter >= `h_front_porch )  && (horizontal_counter < `h_front_porch + `h_sync_pulse)) ? 1'b0 : 1'b1;
   VGA_VS <= ((vertical_counter   >= `v_front_porch )  && (vertical_counter   < `v_front_porch + `v_sync_pulse)) ? 1'b0 : 1'b1;
   
   VGA_DE <= (horizontal_counter >= `h_visible_offset) &&
             (horizontal_counter <  `h_visible_offset_end) &&
             (vertical_counter   >= `v_visible_offset) &&
             (vertical_counter   <  `v_visible_offset_end); 
   
   horizontal_counter <= horizontal_counter + 1'b1;      

   if (horizontal_counter == `h_line_timing) 
   begin
       vertical_counter <= vertical_counter + 1'b1;                
       horizontal_counter <= 11'b0;
   end
   
   if (vertical_counter == `v_line_timing) 
       vertical_counter <= 11'b0;                  
end

endmodule
	
