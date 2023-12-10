
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
  // Interface registers
  reg [31:0] Q;
  reg [31:0] M;

  reg [63:0] input_q_array_real[15:0];
  reg [63:0] input_q_array_img[15:0];
  reg [63:0] temp_real[15:0];
  reg [63:0] temp_img[15:0];

  reg                                                q_state_input_sram_write_enable_r ; 
  reg [`Q_STATE_INPUT_SRAM_ADDRESS_UPPER_BOUND-1:0]  q_state_input_sram_write_address_r; 
  reg [`Q_STATE_INPUT_SRAM_DATA_UPPER_BOUND-1:0]     q_state_input_sram_write_data_r   ; 
  reg [`Q_STATE_INPUT_SRAM_ADDRESS_UPPER_BOUND-1:0]  q_state_input_sram_read_address_r ; 

  reg [63:0] q_gates_real[5119:0];
  reg [63:0] q_gates_img[5119:0];

  reg                                                q_gates_sram_write_enable_r ; 
  reg  [`Q_GATES_SRAM_ADDRESS_UPPER_BOUND-1:0]       q_gates_sram_write_address_r; 
  reg  [`Q_GATES_SRAM_DATA_UPPER_BOUND-1:0]          q_gates_sram_write_data_r   ; 
  reg  [`Q_GATES_SRAM_ADDRESS_UPPER_BOUND-1:0]       q_gates_sram_read_address_r ;  

  reg [63:0] output_q_array_real[15:0];
  reg [63:0] output_q_array_img[15:0];

  reg                                                q_state_output_sram_write_enable_r ; 
  reg [`Q_STATE_OUTPUT_SRAM_ADDRESS_UPPER_BOUND-1:0] q_state_output_sram_write_address_r; 
  reg [`Q_STATE_OUTPUT_SRAM_DATA_UPPER_BOUND-1:0]    q_state_output_sram_write_data_r   ; 
  reg [`Q_STATE_OUTPUT_SRAM_ADDRESS_UPPER_BOUND-1:0] q_state_output_sram_read_address_r ; 

/**************************************************************************************************/
  // Parameters for DW_fp_mac
  localparam inst_sig_width = 52;
  localparam inst_exp_width = 11;
  localparam inst_ieee_compliance = 1;

  reg  [inst_sig_width+inst_exp_width : 0] inst_a1;
  reg  [inst_sig_width+inst_exp_width : 0] inst_b1;
  reg  [inst_sig_width+inst_exp_width : 0] inst_c1;
  reg  [2 : 0] inst_rnd1 = 3'b0;
  wire [inst_sig_width+inst_exp_width : 0] z_inst1;
  wire [7 : 0] status_inst1;

  reg  [inst_sig_width+inst_exp_width : 0] inst_a2;
  reg  [inst_sig_width+inst_exp_width : 0] inst_b2;
  reg  [inst_sig_width+inst_exp_width : 0] inst_c2;
  reg  [2 : 0] inst_rnd2 = 3'b0;
  wire [inst_sig_width+inst_exp_width : 0] z_inst2;
  wire [7 : 0] status_inst2;
/**************************************************************************************************/
  // State parameters
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

  // Conditional variables
  reg         dut_ready_r         ;
  reg         dut_complete        ;
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
  reg         resetJ              ;

  // Counters for copying and computing
  integer i;
  integer counter;
  reg [31:0] N, I, J;
  reg Logic;

  // Current state
  always @(posedge clk) begin
    if(!reset_n) begin // Synchronous reset
      current_state <= IDLE;
    end else begin
      current_state <= next_state;
    end
  end

  // FSM Structure
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
          dut_complete       = 1'b0;
          next_state          = s1;
        end
        else begin
          dut_complete       = 1'b1;
          next_state          = IDLE;
        end
      end

      // Load array size Q and M
      s1: begin
        dut_complete         = 1'b0;
        get_array_size        = 1'b1;   // Get Q and M 
        read_addr_sel         = 2'b11;  // Initialize the read addr
        load_input_state      = 1'b0;
        load_gates            = 1'b0;
        compute_accumulation  = 1'b0;
        save_array_size       = 1'b0;
        write_enable_sel      = 1'b0;
        next_state            = s2;
      end 

      // Store array size Q and M
      s2: begin
        dut_complete         = 1'b0;
        get_array_size        = 1'b0;
        read_addr_sel         = 2'b01;  // Hold the read addr
        load_input_state      = 1'b0;
        load_gates            = 1'b0;
        compute_accumulation  = 1'b0;
        save_array_size       = 1'b1;
        write_enable_sel      = 1'b0;
        next_state            = s3;    
      end

      // Load and store state values
      s3: begin
        dut_complete         = 1'b0;
        get_array_size        = 1'b0;
        read_addr_sel         = 2'b01;  // Keep incrementing the read addr
        load_input_state      = 1'b1;   // Load state value
        load_gates            = 1'b0;
        compute_accumulation  = 1'b0;
        save_array_size       = 1'b1;
        write_enable_sel      = 1'b0;
        next_state            = load_state_completed ? s4 : s3;
      end 

      // Load and store gate values
      s4: begin
        dut_complete         = 1'b0;
        get_array_size        = 1'b0;
        read_addr_sel         = 2'b01;  // Keep incrementing the read addr
        load_input_state      = 1'b0;
        load_gates            = 1'b1;   // Load gate value
        compute_accumulation  = 1'b0;
        save_array_size       = 1'b1;
        write_enable_sel      = 1'b0;
        next_state            = load_gates_completed ? s5 : s4;
      end 

      // Compute multiplication of matrices
      s5: begin
        dut_complete         = 1'b0;
        get_array_size        = 1'b0;
        read_addr_sel         = 2'b10;  // Hold the address
        load_input_state      = 1'b0;
        load_gates            = 1'b0;
        compute_accumulation  = 1'b1;
        save_array_size       = 1'b1;
        write_enable_sel      = 1'b0;
        next_state            = compute_complete ? s6 : s5;    
      end

      // Write results into SRAM
      s6: begin
        dut_complete         = 1'b0;
        get_array_size        = 1'b0;
        read_addr_sel         = 2'b10;  // Hold the address
        load_input_state      = 1'b0;
        load_gates            = 1'b0;
        compute_accumulation  = 1'b0;
        save_array_size       = 1'b1;
        write_enable_sel      = 1'b1;
        next_state            = write_completed ? s7 : s6;
      end
      
      // Reset dut to ready and back to IDLE
      s7: begin
        dut_complete         = 1'b1;
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
        dut_complete         = 1'b1;
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


  // DUT ready handshake logic
  always @(posedge clk) begin
    if(!reset_n) begin
      dut_ready_r <= 0;
    end else begin
      dut_ready_r <= (dut_complete) ? 1'b1 : 1'b0;
    end
  end

  assign dut_ready = dut_ready_r;

  // Find the value of array size
  always @(posedge clk) begin
    if(current_state == IDLE) begin
      Q <= 0;
      M <= 0;
    end else begin
      Q <= get_array_size ? (1<<q_state_input_sram_read_data[127:64]) : (save_array_size ? Q : 0);
      M <= get_array_size ? q_state_input_sram_read_data[63:0] : (save_array_size ? M : 0);
    end
  end

  // Read input address from SRAM
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
        if (read_addr_sel == 2'b00) begin  // Reset address
          q_state_input_sram_read_address_r <= 0;
          q_gates_sram_read_address_r <= 0;
        end
        else if (read_addr_sel == 2'b01) begin  // Increase by one
          q_state_input_sram_read_address_r <= load_input_state ? (load_state_completed ? q_state_input_sram_read_address_r : q_state_input_sram_read_address_r + 1) : q_state_input_sram_read_address_r;
          q_gates_sram_read_address_r <= load_gates ? (load_gates_completed ? q_gates_sram_read_address_r : q_gates_sram_read_address_r + 1) : q_gates_sram_read_address_r;
        end
        else if (read_addr_sel == 2'b10) begin  // Hold address
          q_state_input_sram_read_address_r <= q_state_input_sram_read_address_r;
          q_gates_sram_read_address_r <= q_gates_sram_read_address_r;
        end
        else if (read_addr_sel == 2'b11) begin // Initialize address
          q_state_input_sram_read_address_r <= 1;
          q_gates_sram_read_address_r <= 0;
        end
      end
  end

  // Load and store value into state array
  always @(posedge clk) begin
    if (current_state == IDLE) begin
      // for (i=0; i < 16; i=i+1) begin
      //   input_q_array_real[i] <= 0;
      //   input_q_array_img[i] <= 0;
      // end
    end 
    else begin
      if (load_input_state) begin
        input_q_array_real[q_state_input_sram_read_address_r-2] <= q_state_input_sram_read_data[127:64];
        input_q_array_img[q_state_input_sram_read_address_r-2] <= q_state_input_sram_read_data[63:0];
      end
    end
  end

  // Load and store value into gate array
  always @(posedge clk) begin
    if (current_state == IDLE) begin
      // for (i=0; i < 5119; i=i+1) begin
      //   q_gates_real[i] <= 0;
      //   q_gates_img[i] <= 0;
      // end
    end 
    else begin
      if (load_gates) begin
        q_gates_real[q_gates_sram_read_address_r-1] <= q_gates_sram_read_data[127:64];
        q_gates_img[q_gates_sram_read_address_r-1] <= q_gates_sram_read_data[63:0];
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

  // Write result address and data to SRAM
  always @(posedge clk) begin
    if (current_state == IDLE) begin
      // for (i=0; i < 16; i=i+1) begin
      //   output_q_array_real[i] <= 0;
      //   output_q_array_img[i] <= 0;
      // end
      q_state_output_sram_read_address_r <= 0;
      q_state_output_sram_write_address_r <= -1;
      q_state_output_sram_write_data_r <= 0;
      counter = 0;
    end
    else begin
      if (write_enable_sel) begin
        q_state_output_sram_write_address_r <= write_completed ? q_state_output_sram_write_address_r : q_state_output_sram_write_address_r + 1;
        q_state_output_sram_write_data_r <= {output_q_array_real[counter], output_q_array_img[counter]};
        counter = write_completed ? counter : counter + 1;
      end
      else begin
        q_state_output_sram_write_address_r <= q_state_output_sram_write_address_r;
        counter = counter;
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
    if (current_state == IDLE) begin
      inst_a1 <= 0;
      inst_b1 <= 0;
      inst_c1 <= 0;
      inst_a2 <= 0;
      inst_b2 <= 0;
      inst_c2 <= 0;
      N <= 0;
      I <= 0;
      J <= 0;
      Logic <= 0;
      resetJ <= 1;
    end 
    else begin
      if (compute_accumulation) begin
          if (resetJ) begin  // Using a condition variable here delays one clock cycle when computing each row, which is easier for me to debug
            if (Logic == 0) begin
              // Real * Real
              inst_a1 <= temp_real[J];
              inst_b1 <= q_gates_real[N*Q*Q+I*Q+J];
              inst_c1 <= z_inst1;
              
              // First Image * Real
              inst_a2 <= temp_img[J];
              inst_b2 <= q_gates_real[N*Q*Q+I*Q+J];
              inst_c2 <= z_inst2;
              Logic <= Logic + 1;
            end
            else if (Logic == 1) begin
              // Image * Image
              // Reverse sign if two image value product
              if (temp_img[J][63] == 1'b0) begin
                inst_a1 <= {1'b1, temp_img[J][62:0]};
              end
              else if (temp_img[J][63] == 1'b1) begin
                inst_a1 <= {1'b0, temp_img[J][62:0]};
              end
              inst_b1 <= q_gates_img[N*Q*Q+I*Q+J];
              inst_c1 <= z_inst1;

              // Second Image * Real
              inst_a2 <= temp_real[J];
              inst_b2 <= q_gates_img[N*Q*Q+I*Q+J];
              inst_c2 <= z_inst2;
              Logic <= 0;

              J <= J + 1;
            end
            
            // Assign result of each row to output array
            if (J == Q) begin
              output_q_array_real[I] <= z_inst1;
              output_q_array_img[I] <= z_inst2;
              inst_c1 <= 0;
              inst_c2 <= 0;
              I <= I + 1;
              J <= -1;
              resetJ <= 0;
            end
          end
          else begin  // Formal computational logic begins
            if (J == -1) begin
              inst_a1 <= temp_real[J+1];
              inst_a2 <= temp_img[J+1];
              inst_b1 <= q_gates_real[N*Q*Q+I*Q+J+1];
              inst_b2 <= q_gates_real[N*Q*Q+I*Q+J+1];
              inst_c1 <= 0;
              inst_c2 <= 0;

              J <= J + 1;
            end
            else begin
              if (Logic == 0) begin
                // Real * Real
                inst_a1 <= temp_real[J];
                inst_b1 <= q_gates_real[N*Q*Q+I*Q+J];
                inst_c1 <= z_inst1;

                // First Image * Real
                inst_a2 <= temp_img[J];
                inst_b2 <= q_gates_real[N*Q*Q+I*Q+J];
                inst_c2 <= z_inst2;
                Logic <= Logic + 1;
              end
              else if (Logic == 1) begin
                // Image * Image
                // Reverse sign if two image value product
                if (temp_img[J][63] == 1'b0) begin
                  inst_a1 <= {1'b1, temp_img[J][62:0]};
                end
                else if (temp_img[J][63] == 1'b1) begin
                  inst_a1 <= {1'b0, temp_img[J][62:0]};
                end
                inst_b1 <= q_gates_img[N*Q*Q+I*Q+J];
                inst_c1 <= z_inst1;

                // Second Image * Real
                inst_a2 <= temp_real[J];
                inst_b2 <= q_gates_img[N*Q*Q+I*Q+J];
                inst_c2 <= z_inst2;
                Logic <= 0;

                J <= J + 1;
              end
            end

            // Assign result of each row to output array
            if (J == Q) begin
              output_q_array_real[I] <= z_inst1;
              output_q_array_img[I] <= z_inst2;
              inst_c1 <= 0;
              inst_c2 <= 0;
              I <= I + 1;
              J <= -1;
            end
          end

          // Refresh the value in temp for computing next gate matrix
          if (I == Q) begin
            for (i=0; i < Q; i=i+1) begin
              temp_real[i] <= output_q_array_real[i];
              temp_img[i] <= output_q_array_img[i];
            end 

            inst_a1 <= output_q_array_real[0];
            inst_a2 <= output_q_array_img[0];
            N <= N + 1;
            I <= 0;
          end
      end
      else begin
        for (i=0; i < Q; i=i+1) begin
            temp_real[i] <= input_q_array_real[i];
            temp_img[i] <= input_q_array_img[i];
        end 
      end
    end
  end

  // Computation complete check
  always @(posedge clk) begin
    if(!reset_n) begin
      compute_complete <= 0;
    end else begin
      compute_complete <= (N == M)  ? 1'b1 : 1'b0;
    end
  end

  // Net connection
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
    inst_a1,
    inst_b1,
    inst_c1,
    inst_rnd1,
    z_inst1,
    status_inst1
  );

  DW_fp_mac_inst FP_MAC2 ( 
    inst_a2,
    inst_b2,
    inst_c2,
    inst_rnd2,
    z_inst2,
    status_inst2
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

