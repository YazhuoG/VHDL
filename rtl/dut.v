
`include "defines.vh"
//---------------------------------------------------------------------------
// DUT 
//---------------------------------------------------------------------------
module MyDesign(
//---------------------------------------------------------------------------
//System signals
  input wire reset_n                      ,  
  input wire clk                          ,

//---------------------------------------------------------------------------
//Control signals
  input wire dut_valid                    , 
  output wire dut_ready                   ,

//---------------------------------------------------------------------------
//q_state_input SRAM interface
  output wire                                               q_state_input_sram_write_enable  ,
  output wire [`Q_STATE_INPUT_SRAM_ADDRESS_UPPER_BOUND-1:0] q_state_input_sram_write_address ,
  output wire [`Q_STATE_INPUT_SRAM_DATA_UPPER_BOUND-1:0]    q_state_input_sram_write_data    ,
  output wire [`Q_STATE_INPUT_SRAM_ADDRESS_UPPER_BOUND-1:0] q_state_input_sram_read_address  , 
  input  wire [`Q_STATE_INPUT_SRAM_DATA_UPPER_BOUND-1:0]    q_state_input_sram_read_data     ,

//---------------------------------------------------------------------------
//q_state_output SRAM interface
  output wire                                                q_state_output_sram_write_enable  ,
  output wire [`Q_STATE_OUTPUT_SRAM_ADDRESS_UPPER_BOUND-1:0] q_state_output_sram_write_address ,
  output wire [`Q_STATE_OUTPUT_SRAM_DATA_UPPER_BOUND-1:0]    q_state_output_sram_write_data    ,
  output wire [`Q_STATE_OUTPUT_SRAM_ADDRESS_UPPER_BOUND-1:0] q_state_output_sram_read_address  , 
  input  wire [`Q_STATE_OUTPUT_SRAM_DATA_UPPER_BOUND-1:0]    q_state_output_sram_read_data     ,

//---------------------------------------------------------------------------
//scratchpad SRAM interface                                                       
  output wire                                                scratchpad_sram_write_enable        ,
  output wire [`SCRATCHPAD_SRAM_ADDRESS_UPPER_BOUND-1:0]     scratchpad_sram_write_address       ,
  output wire [`SCRATCHPAD_SRAM_DATA_UPPER_BOUND-1:0]        scratchpad_sram_write_data          ,
  output wire [`SCRATCHPAD_SRAM_ADDRESS_UPPER_BOUND-1:0]     scratchpad_sram_read_address        , 
  input  wire [`SCRATCHPAD_SRAM_DATA_UPPER_BOUND-1:0]        scratchpad_sram_read_data           ,

//---------------------------------------------------------------------------
//q_gates SRAM interface                                                       
  output wire                                                q_gates_sram_write_enable           ,
  output wire [`Q_GATES_SRAM_ADDRESS_UPPER_BOUND-1:0]        q_gates_sram_write_address          ,
  output wire [`Q_GATES_SRAM_DATA_UPPER_BOUND-1:0]           q_gates_sram_write_data             ,
  output wire [`Q_GATES_SRAM_ADDRESS_UPPER_BOUND-1:0]        q_gates_sram_read_address           ,  
  input  wire [`Q_GATES_SRAM_DATA_UPPER_BOUND-1:0]           q_gates_sram_read_data              
);

  
/**************************************************************************************************/
  reg [31:0] Q;
  reg [31:0] M;

  reg [63:0] input_q_array_real[3:0];
  reg [63:0] input_q_array_img[3:0];

  reg                                                q_state_input_sram_write_enable_r ; 
  reg [`Q_STATE_INPUT_SRAM_ADDRESS_UPPER_BOUND-1:0]  q_state_input_sram_write_address_r; 
  reg [`Q_STATE_INPUT_SRAM_DATA_UPPER_BOUND-1:0]     q_state_input_sram_write_data_r   ; 
  reg [`Q_STATE_INPUT_SRAM_ADDRESS_UPPER_BOUND-1:0]  q_state_input_sram_read_address_r ; 

  reg [63:0] q_gates_real[319:0];
  reg [63:0] q_gates_img[319:0];

  reg                                                q_gates_sram_write_enable_r ; 
  reg  [`Q_GATES_SRAM_ADDRESS_UPPER_BOUND-1:0]       q_gates_sram_write_address_r; 
  reg  [`Q_GATES_SRAM_DATA_UPPER_BOUND-1:0]          q_gates_sram_write_data_r   ; 
  reg  [`Q_GATES_SRAM_ADDRESS_UPPER_BOUND-1:0]       q_gates_sram_read_address_r ;  

  reg [63:0] output_q_array_real[3:0];
  reg [63:0] output_q_array_img[3:0];

  reg                                                q_state_output_sram_write_enable_r ; 
  reg [`Q_STATE_OUTPUT_SRAM_ADDRESS_UPPER_BOUND-1:0] q_state_output_sram_write_address_r; 
  reg [`Q_STATE_OUTPUT_SRAM_DATA_UPPER_BOUND-1:0]    q_state_output_sram_write_data_r   ; 
  reg [`Q_STATE_OUTPUT_SRAM_ADDRESS_UPPER_BOUND-1:0] q_state_output_sram_read_address_r ; 

/**************************************************************************************************/
//Parameters for DW_fp_mac
  localparam inst_sig_width = 52;
  localparam inst_exp_width = 11;
  localparam inst_ieee_compliance = 1;

  reg  [inst_sig_width+inst_exp_width : 0] inst_a;
  reg  [inst_sig_width+inst_exp_width : 0] inst_b;
  reg  [inst_sig_width+inst_exp_width : 0] inst_c;
  reg  [2 : 0] inst_rnd = 3'b0;
  wire [inst_sig_width+inst_exp_width : 0] z_inst;
  wire [7 : 0] status_inst;
/**************************************************************************************************/

  parameter [2:0]
    IDLE  = 3'd0, 
    s1   = 3'd1,   
    s2   = 3'd2,   
    s3   = 3'd3,   
    s4   = 3'd4,   
    s5   = 3'd5,   
    s6   = 3'd6,
    s7   = 3'd7;

  reg [2:0]		current_state;	//FSM current state
	reg [2:0]		next_state;	//FSM next state

  reg         dut_ready_r         ;
  reg         get_array_size      ;
  reg [1:0]   read_addr_sel       ;
  reg         load_input_state    ;
  reg         load_state_completed;
  reg         load_gates          ;
  reg         load_gates_completed;
  reg         compute_accumulation;
  reg         save_array_size     ;
  reg         write_enable_sel    ;
  reg         write_completed     ;
  reg         compute_complete    ;

  integer m, i, j;
  integer counter1, counter2, counter3;
  integer N, I, J;

  always @(posedge clk) begin
    if(!reset_n) begin // Synchronous reset
      current_state <= IDLE;
    end else begin
      current_state <= next_state;
    end
  end


  always @(*) begin
    case (current_state)

      IDLE: begin
        get_array_size      = 1'b0;
        read_addr_sel       = 2'b00;
        load_input_state    = 1'b0;
        load_gates          = 1'b0;
        compute_accumulation= 1'b0;
        save_array_size     = 1'b0;
        write_enable_sel    = 1'b0;
        if (dut_valid) begin
          dut_ready_r       = 1'b0;
          next_state          = s1;
        end
        else begin
          dut_ready_r       = 1'b1;
          next_state          = IDLE;
        end
      end
    
      s1: begin
        dut_ready_r         = 1'b0;
        get_array_size        = 1'b1;   // Get Q and M 
        read_addr_sel         = 2'b11;  // Initialize the read addr
        load_input_state      = 1'b0;
        load_gates            = 1'b0;
        compute_accumulation  = 1'b0;
        save_array_size       = 1'b0;
        write_enable_sel      = 1'b0;
        next_state            = s2;
      end 

      s2: begin
        dut_ready_r         = 1'b0;
        get_array_size        = 1'b0;
        read_addr_sel         = 2'b01;  // Hold the read addr
        load_input_state      = 1'b0;
        load_gates            = 1'b0;
        compute_accumulation  = 1'b0;
        save_array_size       = 1'b1;
        write_enable_sel      = 1'b0;
        next_state            = s3;    
      end

      s3: begin
        dut_ready_r         = 1'b0;
        get_array_size        = 1'b0;
        read_addr_sel         = 2'b01;  // Keep incrementing the read addr
        load_input_state      = 1'b1;   // Load state value
        load_gates            = 1'b0;
        compute_accumulation  = 1'b0;
        save_array_size       = 1'b1;
        write_enable_sel      = 1'b0;
        next_state            = load_state_completed ? s4 : s3;
      end 

      s4: begin
        dut_ready_r         = 1'b0;
        get_array_size        = 1'b0;
        read_addr_sel         = 2'b01;  // Keep incrementing the read addr
        load_input_state      = 1'b0;
        load_gates            = 1'b1;   // Load gate value
        compute_accumulation  = 1'b0;
        save_array_size       = 1'b1;
        write_enable_sel      = 1'b0;
        next_state            = load_gates_completed ? s5 : s4;
      end 

      s5: begin
        dut_ready_r         = 1'b0;
        get_array_size        = 1'b0;
        read_addr_sel         = 2'b10;  // Hold the address
        load_input_state      = 1'b0;
        load_gates            = 1'b0;
        compute_accumulation  = 1'b1;
        save_array_size       = 1'b1;
        write_enable_sel      = 1'b0;
        next_state            = compute_complete ? s6 : s5;    
      end

      s6: begin
        dut_ready_r         = 1'b0;
        get_array_size        = 1'b0;
        read_addr_sel         = 2'b10;  // Hold the address
        load_input_state      = 1'b0;
        load_gates            = 1'b0;
        compute_accumulation  = 1'b0;
        save_array_size       = 1'b1;
        write_enable_sel      = 1'b1;
        next_state            = write_completed ? s7 : s6;
      end

      s7: begin
        dut_ready_r         = 1'b1;
        get_array_size        = 1'b0;
        read_addr_sel         = 2'b00;  
        load_input_state      = 1'b0;
        load_gates            = 1'b0;
        compute_accumulation  = 1'b0;
        save_array_size       = 1'b0;
        write_enable_sel      = 1'b0;
        next_state            = IDLE;      
      end

      default:  begin
        dut_ready_r         = 1'b1;
        get_array_size        = 1'b0;
        read_addr_sel         = 2'b00;  
        load_input_state      = 1'b0;
        load_gates            = 1'b0;
        compute_accumulation  = 1'b0;
        save_array_size       = 1'b0;
        write_enable_sel      = 1'b0;      
        next_state            = IDLE;
      end
    endcase
  end


  // // DUT ready handshake logic
  // always @(posedge clk) begin
  //   if(!reset_n) begin
  //     compute_complete <= 0;
  //   end else begin
  //     compute_complete <= (dut_ready_r) ? 1'b1 : 1'b0;
  //   end
  // end

  assign dut_ready = dut_ready_r;

  // Find the number of array elements 
  always @(posedge clk) begin
    if(!reset_n) begin
      Q <= 0;
      M <= 0;
    end else begin
      Q <= get_array_size ? (1<<q_state_input_sram_read_data[127:64]) : (save_array_size ? Q : 0);
      M <= get_array_size ? q_state_input_sram_read_data[63:0] : (save_array_size ? M : 0);
    end
  end

  // SRAM read address generator
  always @(posedge clk) begin
      if (!reset_n) begin
        q_state_input_sram_write_enable_r   <= 0;
        q_state_input_sram_write_address_r  <= 0;
        q_state_input_sram_write_data_r     <= 0;
        q_state_input_sram_read_address_r   <= 0;
        q_gates_sram_write_enable_r   <= 0;
        q_gates_sram_write_address_r  <= 0;
        q_gates_sram_write_data_r     <= 0;
        q_gates_sram_read_address_r   <= 0;
      end
      else begin
        if (read_addr_sel == 2'b00) begin
          q_state_input_sram_read_address_r <= 0;
          q_gates_sram_read_address_r <= 0;
        end
        else if (read_addr_sel == 2'b01) begin
          q_state_input_sram_read_address_r <= load_input_state ? (load_state_completed ? q_state_input_sram_read_address_r : q_state_input_sram_read_address_r + 1) : q_state_input_sram_read_address_r;
          q_gates_sram_read_address_r <= load_gates ? (load_gates_completed ? q_gates_sram_read_address_r : q_gates_sram_read_address_r + 1) : q_gates_sram_read_address_r;
        end
        else if (read_addr_sel == 2'b10) begin
          q_state_input_sram_read_address_r <= q_state_input_sram_read_address_r;
          q_gates_sram_read_address_r <= q_gates_sram_read_address_r;
        end
        else if (read_addr_sel == 2'b11) begin
          q_state_input_sram_read_address_r <= 1;
          q_gates_sram_read_address_r <= 0;
        end
      end
  end

  // Load and store value into state array
  always @(posedge clk) begin
    if (!reset_n) begin
      for (i=0; i < Q; i=i+1) begin
        input_q_array_real[i] <= 0;
        input_q_array_img[i] <= 0;
      end
      counter1 = -1;
    end 
    else begin
      if (load_input_state) begin
        input_q_array_real[counter1] <= q_state_input_sram_read_data[127:4];
        // output_q_array_img[counter1] <= load_input_state ? q_state_input_sram_read_data[63:0] : 0;
        counter1 = counter1 + 1;
      end
    end
  end

  // Load and store value into gate array
  always @(posedge clk) begin
    if (!reset_n) begin
      for (m=0; m < M; m=m+1) begin
        for (i=0; i < Q; i=i+1) begin
          for (j=0; j < Q; j=j+1) begin
            q_gates_real[m*Q*Q+i*Q+j] <= 0;
            q_gates_img[m*Q*Q+i*Q+j] <= 0;
          end
        end
      end
      counter2 = -1;
    end 
    else begin
      if (load_gates) begin
        q_gates_real[counter2] <= q_gates_sram_read_data[127:4];
        // q_gates_img[q_gates_sram_read_address_r] <= load_gates ? q_gates_sram_read_data[63:0] : 0;
        counter2 = counter2 + 1;
      end
    end
  end

  // Load complete check 
  always @(posedge clk) begin
    if (!reset_n) begin
      load_state_completed <= 1'b0;
    end 
    else begin
      load_state_completed <= (q_state_input_sram_read_address_r  == Q) ? 1'b1 : 1'b0;
    end
  end

  // Load complete check
  always @(posedge clk) begin
    if (!reset_n) begin
      load_gates_completed <= 1'b0;
    end 
    else begin
      load_gates_completed <= (q_gates_sram_read_address_r  == Q*Q*M-1) ? 1'b1 : 1'b0;
    end
  end

  // SRAM write enable logic
  always @(posedge clk) begin
    if (!reset_n) begin
      q_state_output_sram_write_enable_r <= 1'b0;
    end 
    else begin
      q_state_output_sram_write_enable_r <= write_enable_sel ? 1'b1 : 1'b0;
    end
  end

  // SRAM write address logic
  always @(posedge clk) begin
    if (!reset_n) begin
      q_state_output_sram_read_address_r <= 0;
      q_state_output_sram_write_address_r <= -1;
      q_state_output_sram_write_data_r <= 0;
      counter3 = 0;
    end
    else begin
      if (write_enable_sel) begin
        q_state_output_sram_write_address_r <= write_completed ? q_state_output_sram_write_address_r : q_state_output_sram_write_address_r + 1;
        q_state_output_sram_write_data_r <= {output_q_array_real[counter3], 64'b0};
        counter3 = write_completed ? counter3 : counter3 + 1;
      end
      else begin
        q_state_output_sram_write_address_r <= q_state_output_sram_write_address_r;
        counter3 = counter3;
      end
    end
  end

  // Write complete check
  always @(posedge clk) begin
    if (!reset_n) begin
      write_completed <= 1'b0;
    end 
    else begin
      write_completed <= (q_state_output_sram_write_address_r == Q-1) ? 1'b1 : 1'b0;
    end
  end


  // Accumulation logic 
  always @(posedge clk) begin
    if (!reset_n) begin
      inst_a <= 64'b0;
      inst_b <= 64'b0;
      inst_c <= 64'b0;
      N = 0;
      I = 0;
      J = 0;
    end 
    else begin
      if (compute_accumulation) begin
        inst_a <= input_q_array_real[J];
        inst_b <= q_gates_real[N*Q*Q+I*Q+J];
        inst_c <= z_inst;
        J = J + 1;

        if (J == Q) begin
          output_q_array_real[I] <= z_inst;
          I = I + 1;
          J = 0;
        end

        if (I == Q) begin
          input_q_array_real[Q-1] <= z_inst;
          for (i=0; i < Q-1; i=i+1) begin
            input_q_array_real[i] <= output_q_array_real[i];
          end 
          
          N = N + 1;
          I = 0;
        end

        // for (m=0; m < M; m=m+1) begin
        //   for (i=0; i < Q; i=i+1) begin
        //     for (j=0; j < Q; j=j+1) begin
        //       inst_a <= input_q_array_real[j];
        //       inst_b <= q_gates_real[m*Q*Q+i*Q+j];
        //       inst_c <= z_inst;
        //     end

        //     output_q_array_real[i] <= inst_c;
        //   end

        //   for (i=0; i < Q; i=i+1) begin
        //     input_q_array_real[i] <= output_q_array_real[i];
        //   end 
        // end
      end
      else begin

      end
    end
  end

  always @(posedge clk) begin
    if(!reset_n) begin
      compute_complete <= 0;
    end else begin
      compute_complete <= (N == M)  ? 1'b1 : 1'b0;
    end
  end

  assign q_state_input_sram_write_enable  = q_state_input_sram_write_enable_r ; 
  assign q_state_input_sram_write_address = q_state_input_sram_write_address_r; 
  assign q_state_input_sram_write_data    = q_state_input_sram_write_data_r   ; 
  assign q_state_input_sram_read_address  = q_state_input_sram_read_address_r ;  

  assign q_gates_sram_write_enable   = q_gates_sram_write_enable_r  ;
  assign q_gates_sram_write_address  = q_gates_sram_write_address_r ;
  assign q_gates_sram_write_data     = q_gates_sram_write_data_r    ;
  assign q_gates_sram_read_address   = q_gates_sram_read_address_r  ;

  assign q_state_output_sram_write_enable   = q_state_output_sram_write_enable_r ; 
  assign q_state_output_sram_write_address  = q_state_output_sram_write_address_r; 
  assign q_state_output_sram_write_data     = q_state_output_sram_write_data_r   ; 
  assign q_state_output_sram_read_address   = q_state_output_sram_read_address_r ;

  // assign scratchpad_sram_write_enable = 0;
  // assign scratchpad_sram_write_address = 0;
  // assign scratchpad_sram_write_data = 0;
  // assign scratchpad_sram_read_address = 0;

  // This is test stub for passing input/outputs to a DP_fp_mac, there many
  // more DW macros that you can choose to use
  DW_fp_mac_inst FP_MAC1 ( 
    inst_a,
    inst_b,
    inst_c,
    inst_rnd,
    z_inst,
    status_inst
  );

endmodule


module DW_fp_mac_inst #(
  parameter inst_sig_width = 52,
  parameter inst_exp_width = 11,
  parameter inst_ieee_compliance = 1 // These need to be fixed to decrease error
) ( 
  input wire [inst_sig_width+inst_exp_width : 0] inst_a,
  input wire [inst_sig_width+inst_exp_width : 0] inst_b,
  input wire [inst_sig_width+inst_exp_width : 0] inst_c,
  input wire [2 : 0] inst_rnd,
  output wire [inst_sig_width+inst_exp_width : 0] z_inst,
  output wire [7 : 0] status_inst
);

  // Instance of DW_fp_mac
  DW_fp_mac #(inst_sig_width, inst_exp_width, inst_ieee_compliance) U1 (
    .a(inst_a),
    .b(inst_b),
    .c(inst_c),
    .rnd(inst_rnd),
    .z(z_inst),
    .status(status_inst) 
  );

endmodule

