//==================================================================================================
//  Filename      : NN_ASIC_0.v
//  Created On    : 2017-12-01 17:31:57
//  Last Modified : 2018-01-22 23:04:29
//  Revision      : 
//  Author        : LIN YUEJIN, Zheng Wang
//  Email         : 1639838189@qq.com zheng.wang@siat.ac.cn
//
//  Description   : 
//
//
//==================================================================================================
`define use_xilinx_ip

module NN_ASIC (
  input       signed[7:0]     in_data,
  input       signed[7:0]     in_CFG,
  input                       in_config_en,
  input                       in_weight_en,
  input                       in_action_en,
  input                       in_reward_en,
  input                       in_data_en,
  input                       rst,
  input                       clk,                            //main clock
  output                      out_cfg_done,                   //debugging
  output                      out_parity,                     //debugging
  output      reg signed[7:0] out_data,                       //data 
  output      reg             out_wea_data,                   //current data complete processing
  output      reg signed[9:0] out_addr,
  output      reg             out_data_received,
  output      reg             out_error
);

//-------------Constants-----------------------
  parameter   neuron_data_size      =8;
  parameter   neuron_array_size     =128;
  parameter   neuron_ram_size       =1024;
  parameter   cfg_ram_size          =1024;
  parameter   act_ram_size          =1024;
  parameter   rewa_ram_size         =1024;
  parameter   cfg_reg_size          =64;
  parameter   neuron_ram_addr_size  =11;
  parameter   ram_cfg_addr_size     =10;
  parameter   ram_cfg_data_size     =8; 
  parameter   ram_act_data_size     =8; 
  parameter   ram_rewa_data_size    =8; 
  parameter   weight_addr_size      =10;
  parameter   weight_data_size      =8;
  parameter   ram_1_addr_size       =10;
  parameter   ram_1_size            =1024;
  parameter   STATE_RST             =5'b00000,       //0
              STATE_IDLE            =5'b00001,       //1
              CONFIG_CFG_RAM        =5'b00010,       //2
              CONFIG_WEIGHT_RAM     =5'b00011,       //3
              CONFIG_ACTION_RAM     =5'b00100,       //4
              CONFIG_REWARD_RAM     =5'b00101,       //5
              STATE_LOAD_DATA       =5'b00110,       //6
              STATE_LOAD_DATA_DONE  =5'b00111,       //7
              STATE_LOAD_INST       =5'b01000,       //8
              STATE_DECODE_INST     =5'b01001,       //9
              STATE_PROCESS_0       =5'b01010,       //10
              STATE_PROCESS_1       =5'b01011,       //11
              STATE_PROCESS_2       =5'b01100,       //12
              STATE_WRITE           =5'b01101,       //13
              STATE_WRITE_DONE      =5'b01110,       //14
              STATE_BRANCH          =5'b01111,       //15
              STATE_BRANCH_DONE     =5'b10000,       //16
              STATE_UPDATE_INST     =5'b10001,       //17
              STATE_DATA_END        =5'b10010,       //18
              STATE_OUTPUT          =5'b10011,       //19
              STATE_OUTPUT_DONE     =5'b10100,       //20
              STATE_ERROR           =5'b10101;       //21
              STATE_RL              =5'b10110;       //22
  parameter   mode                  =2'b01;                 // update later from config chain
  parameter   threshold             =16'b0000000001010100;

//------physical registers---------------------
  reg   [neuron_data_size-1:0]      REG_data_in          [0:neuron_array_size-1];  //signed type
  reg   [neuron_data_size-1:0]      REG_data_out         [0:neuron_array_size-1];
  reg   [neuron_data_size-1:0]      REG_data_temp        [0:neuron_array_size-1];
  reg   [neuron_ram_addr_size-1:0]  REG_cnt_array;
  reg   [7:0]                       REG_cnt_class;                                 //6 is enough,but when 127+1,turns to zero!
  reg   [7:0]                       REG_cnt_out_nodes;                             //a liitle larger than "128"
  reg   [15:0]                      REG_cnt_total_out_nodes;
  reg   [6:0]                       REG_neuron_array_end;
  reg   [6:0]                       REG_cnt_cfg_reg;  // counter of config register
  reg   [ram_1_addr_size-1:0]       ram_1_addr;
  reg   [ram_cfg_addr_size-1:0]     ram_cfg_addr;
  reg   [ram_act_addr_size-1:0]     ram_act_addr;
  reg   [ram_rewa_addr_size-1:0]    ram_rewa_addr;
  reg   [7:0]                       REG_cnt_PROCESS_2;
  reg   [ram_1_addr_size-1:0]       REG_ram_1_write_beg, REG_ram_1_write_end;                          //the{1th, 3th, 5th, 7th...}layer output restore initial address
  reg   [ram_1_addr_size-1:0]       REG_ram_1_read_beg;
  reg   [cfg_reg_size-1:0]          REG_config_chain;
  reg   [ram_cfg_data_size-1:0]     ram_cfg_write;
  reg   [ram_act_data_size-1:0]     ram_act_write;
  reg   [ram_rewa_data_size-1:0]    ram_rewa_write;
  reg   [neuron_array_size-1:0]     neuron_enable;
  reg   [4:0]                       next_state_ctrl, state_ctrl;
  reg   [4:0]                       REG_cnt_layer;
  reg                               REG_RL_enable;

//------other reg type signals-----------------
  reg  [7:0]                        ram_1_write;
  reg                               web_ram_1;
  reg                               csb_ram_1;
  reg                               web_ram_cfg;
  reg                               csb_ram_cfg;
  reg   [weight_data_size-1:0]      sig_weight_in    [0:neuron_array_size-1]; 
  reg   [neuron_array_size-1:0]     sig_weight_wr;
  reg   [neuron_array_size-1:0]     sig_neuron_clear;
  reg                               sig_config_done_weight;
  reg                               sig_config_done_cfg;
  reg                               sig_config_done_data;
  reg                               sig_error;
  reg                               sig_ram_1_wrap;
  reg  [15:0]                       sig_size_in_out_total;

//------wires----------------------------------
  wire  [7:0]                       ram_1_read;
  wire  [ram_cfg_data_size-1:0]     ram_cfg_read;
  wire  [ram_act_data_size-1:0]     ram_act_read;
  wire  [ram_rewa_data_size-1:0]    ram_rewa_read;
  wire  [neuron_data_size-1:0]      sig_output_data [0:neuron_array_size-1];
  wire  [15:0]                      var_layer_in_nodes;
  wire  [15:0]                      var_layer_out_nodes;
  wire  [7:0]                       var_layer_depth;
  wire  [2:0]                       var_layer_type;
  wire                              var_layer_precision;
  wire                              var_layer_activation;
  wire                              var_layer_first;
  wire                              var_layer_done;
  wire                              var_layer_reuse;

//===========================logic blocks===========================
//----------0. assignment of config_chain---------------------
  assign  var_layer_out_nodes  = REG_config_chain[15:0];
  assign  var_layer_in_nodes   = REG_config_chain[31:16];
  assign  var_layer_depth      = REG_config_chain[39:32];
  assign  var_layer_type       = REG_config_chain[42:40];
  assign  var_layer_precision  = REG_config_chain[43];
  assign  var_layer_activation = REG_config_chain[44];
  assign  var_layer_first      = REG_config_chain[45];
  assign  var_layer_done       = REG_config_chain[46];
  assign  var_layer_reuse      = REG_config_chain[47];

//----------1. top level controller FSM---------------------
  always @ (*)
  begin: FSM_next_state_ctrl
    //next_state_ctrl = 3'bxxx;
    case(state_ctrl)
      STATE_RST : //Reset the data path 
                next_state_ctrl = STATE_IDLE; 
      STATE_IDLE : //Data path idle state
                if (in_data_en)             // storing data from interface sectionwise
                  next_state_ctrl = STATE_LOAD_DATA; 
                else if (in_config_en)                 //pulse trriggle
                  next_state_ctrl = CONFIG_CFG_RAM;
                else if (in_weight_en)
                  next_state_ctrl = CONFIG_WEIGHT_RAM;  
                else if (in_action_en)
                  next_state_ctrl = CONFIG_ACTION_RAM;  
                else if (in_reward_en)
                  next_state_ctrl = CONFIG_REWARD_RAM;  
                else
                  next_state_ctrl = STATE_IDLE;
      CONFIG_CFG_RAM : 
               if (sig_config_done_cfg)          // internal regs
                  next_state_ctrl = STATE_LOAD_INST;
               else
                  next_state_ctrl = CONFIG_CFG_RAM;
      CONFIG_WEIGHT_RAM : 
               if (sig_config_done_weight)            // config neuron-wise weight memories
                  next_state_ctrl = STATE_IDLE;
               else
                  next_state_ctrl = CONFIG_WEIGHT_RAM;
      CONFIG_ACTION_RAM : 
               if (sig_config_done_action)            // config neuron-wise weight memories
                  next_state_ctrl = STATE_IDLE;
               else
                  next_state_ctrl = CONFIG_ACTION_RAM;
      CONFIG_REWARD_RAM : 
               if (sig_config_done_reward)            // config neuron-wise weight memories
                  next_state_ctrl = STATE_IDLE;
               else
                  next_state_ctrl = CONFIG_REWARD_RAM;
      STATE_LOAD_DATA : //storing data from interface
                if (sig_config_done_data)   //stop when all input nodes are filled
                  next_state_ctrl = STATE_LOAD_DATA_DONE;
                else
                  next_state_ctrl = STATE_LOAD_DATA;
      STATE_LOAD_DATA_DONE : 
                  next_state_ctrl = STATE_PROCESS_0;
      STATE_LOAD_INST : //load layer-wise configuration bits to configuration chain registers
                if (REG_cnt_cfg_reg>=cfg_reg_size-1)  //step size is not 1 but 8 here
                  next_state_ctrl = STATE_DECODE_INST;
                else
                  next_state_ctrl = STATE_LOAD_INST;
      STATE_DECODE_INST : 
                if (sig_error)
                  next_state_ctrl = STATE_ERROR;
                else if (var_layer_first)
                  next_state_ctrl = STATE_IDLE;
                else
                begin
                  case(var_layer_type)
                    3'b000 : 
                    begin
                      next_state_ctrl = STATE_PROCESS_0;
                    end
                   // 3'b001 : 
                   // begin
                   //   next_state_ctrl = STATE_CONV;
                   // end
                    3'b010 : 
                    begin
                      next_state_ctrl = STATE_RL;
                    end
                   // 3'b011 : 
                   // begin
                   //   next_state_ctrl = STATE_RNN;
                   // end
                    default:
                    begin
                      next_state_ctrl = STATE_ERROR;
                    end
                  endcase
                end
      STATE_PROCESS_0 : //ram read data shift to the input registers   // need debug Zheng Wang
                  next_state_ctrl = STATE_PROCESS_1;
      STATE_PROCESS_1 : //ram read data shift to the input registers   // need debug Zheng Wang
                if (ram_1_addr>REG_ram_1_read_beg + var_layer_in_nodes[ram_1_addr_size-1:0]-1)  // REG_input_nodes: total input nodes count for current network layer
                  next_state_ctrl = STATE_PROCESS_2;                           //perhanps the REG_cnt_in_nodes=REG_total_in_nodes + 128 or more wait for the result from the last neuron
                else
                  next_state_ctrl = STATE_PROCESS_1;
      STATE_PROCESS_2 : //determine when the sig_output_data write to the output registers
                if (REG_cnt_PROCESS_2>=REG_neuron_array_end + 2)  // branch condition has been changed : 2 for delayed cycles to get all data back from neurons
                begin
                  if (~REG_RL_enable)
                    next_state_ctrl = STATE_WRITE;
                  else
                    next_state_ctrl = STATE_ACTION;
                end
                else
                  next_state_ctrl = STATE_PROCESS_2;
      STATE_WRITE : //shifting output registers' data to ram 
                if (REG_cnt_out_nodes>=REG_neuron_array_end-1)
                  next_state_ctrl = STATE_WRITE_DONE;
                else
                  next_state_ctrl = STATE_WRITE;
      STATE_WRITE_DONE :
                  next_state_ctrl = STATE_BRANCH;
      STATE_BRANCH : //a full output node block is done, and turns to next block
                if (REG_cnt_total_out_nodes>=var_layer_out_nodes-1)
                  next_state_ctrl = STATE_BRANCH_DONE; 
                else
                  next_state_ctrl = STATE_PROCESS_0;
      STATE_BRANCH_DONE :
                  next_state_ctrl = STATE_UPDATE_INST; 
      STATE_UPDATE_INST :
                if (var_layer_done)
                  next_state_ctrl = STATE_DATA_END;
                else
                  next_state_ctrl = STATE_LOAD_INST;
      STATE_DATA_END : 
                  next_state_ctrl = STATE_OUTPUT;
      STATE_OUTPUT :
                if (ram_1_addr>=REG_ram_1_write_end-1)              //output to OK RAM debug by Wang Zheng
                  next_state_ctrl = STATE_OUTPUT_DONE;
                else
                  next_state_ctrl = STATE_OUTPUT;                           //with doubt whether STATE_PROCESS_1 or STATE_IDLE
      STATE_OUTPUT_DONE :  next_state_ctrl = STATE_IDLE; 
      STATE_RL :   next_state_ctrl = STATE_PROCESS_0; 
      STATE_ACTION :  next_state_ctrl = STATE_ACTION_READ_1; 
      STATE_ACTION_READ_1 :  next_state_ctrl = STATE_ACTION_READ_2; 
      STATE_ACTION_READ_2 :  next_state_ctrl = STATE_ACTION_READ_3; 
      STATE_ACTION_READ_3 :
                if (ram_act_addr>=REG_ACT_total-1)
                  next_state_ctrl = STATE_ACTION_DONE;
                else
                  next_state_ctrl = STATE_ACTION_READ_1;
      STATE_ACTION_DONE :
                next_state_ctrl = STATE_PROCESS_2; 
      STATE_ERROR :   next_state_ctrl = STATE_IDLE; 
      default :   next_state_ctrl = STATE_IDLE;                             //synthis compare the difference between default and initial
    endcase
  end

  always @ (posedge clk)
  begin: FSM_state_regs_ctrl
    integer i;
    if (!rst)
      state_ctrl <= STATE_RST;
    else 
    begin
      state_ctrl <= next_state_ctrl;                                   //state change will delay a clock edge!!!
      case(state_ctrl)                                                //at the second clock edge, the signal start changing!!! totaly cost two clock edges
        STATE_RST :
        begin
          REG_cnt_layer<=0;
          REG_cnt_out_nodes<=0;
          REG_cnt_total_out_nodes<=0;
          ram_1_addr       <=0;                                               //perhaps should set as "-1"
          REG_ram_1_write_beg<=0;                            //generate the offset address"512"
          REG_ram_1_write_end<=0;                            //generate the offset address"512"
          REG_ram_1_read_beg <=0;  
          for(i=0;i<=neuron_array_size-1;i=i+1)
          begin
              neuron_enable[i]<=0;
              REG_data_in[i]  <=0;
              REG_data_out[i] <=0;
              REG_data_temp[i]<=0;
          end
          out_data_received<=0;
          REG_cnt_array<=0;
          REG_cnt_class<=0;
          ram_cfg_write<=0;
          ram_cfg_addr <=0; // resolve data lost at address 0
          ram_act_addr <=0; 
          ram_rewa_addr<=0;
          REG_cnt_cfg_reg<=0;
          REG_cnt_PROCESS_2<=0;
          REG_config_chain<=0;
          REG_neuron_array_end<=0;
          REG_RL_enable<=0;
          out_addr<=0;
          out_error<=0;
        end
        STATE_IDLE :
        begin
          out_addr<=0;
          REG_cnt_out_nodes<=0;
          REG_cnt_total_out_nodes<=0;
          for(i=0;i<=neuron_array_size-1;i=i+1)
          begin
              neuron_enable[i]<=0;
              REG_data_in[i]  <=0;
              REG_data_out[i] <=0;
          end
          REG_cnt_array<=0;
          REG_cnt_class<=0;
          REG_cnt_cfg_reg<=0;
          REG_cnt_PROCESS_2<=0;
          REG_neuron_array_end<=0;
          REG_RL_enable<=0;
          if (in_config_en)
            ram_cfg_write <= in_CFG;
          else
            ram_cfg_write <= 0;
          if (in_weight_en)
            REG_cnt_array<=REG_cnt_array+1'b1; 
        end
        CONFIG_CFG_RAM :
        begin
          if(ram_cfg_addr < cfg_ram_size)
          begin
            ram_cfg_write <= in_CFG;
            ram_cfg_addr  <= ram_cfg_addr + 1'b1;
          end
          else
          begin
            ram_cfg_write <= 0;
            ram_cfg_addr  <= 0;
          end
        end
        CONFIG_WEIGHT_RAM :
        begin
          if (REG_cnt_class < neuron_array_size)                                 //-1
          begin
            if (REG_cnt_array < neuron_ram_size-1)
            begin
              REG_cnt_array<=REG_cnt_array+1'b1;                                 //memory address array
            end
            else
            begin
              REG_cnt_class<=REG_cnt_class+1'b1;                                 //config nex neuron 
              REG_cnt_array<=0;
            end
          end
        end
        STATE_LOAD_DATA : 
        begin
          ram_1_addr<=ram_1_addr+1'b1;                                 //write cycle from outside
          out_data_received<=0;
        end
        STATE_LOAD_DATA_DONE : 
        begin
	  REG_ram_1_read_beg<=0;
          REG_ram_1_write_beg<=ram_1_addr;
          REG_ram_1_write_end<=ram_1_addr;
          ram_1_addr<=0;
        end
        STATE_LOAD_INST : 
        begin
          if (REG_cnt_cfg_reg<cfg_reg_size-1)
          begin
            ram_cfg_addr  <= ram_cfg_addr + 1'b1;
            REG_cnt_cfg_reg<=REG_cnt_cfg_reg+8;
          end
          REG_config_chain<={REG_config_chain[cfg_reg_size-9:0],ram_cfg_read};
        end
        STATE_DECODE_INST :
        begin
          REG_cnt_cfg_reg<=0;
          if (~var_layer_first)  // switch read_beg and write_beg registers for new network layer
          begin
	    REG_ram_1_read_beg<=REG_ram_1_write_beg;
            ram_1_addr<=REG_ram_1_write_beg;
            if (sig_ram_1_wrap)  // if write data need to wrap around, write from very beginning 
            begin
              REG_ram_1_write_beg<=0;
              REG_ram_1_write_end<=0;
            end
            else  // otherwise go on writing after previous layer output 
              REG_ram_1_write_beg<=REG_ram_1_write_end;
          end
        end
        STATE_PROCESS_0: 
        begin
          if (REG_RL_enable) 
          begin
            REG_ACT_total<=ram_act_read;
          end
          ram_1_addr    <=ram_1_addr+1'b1;       
          REG_cnt_PROCESS_2 <= 0;
          if ((var_layer_out_nodes-REG_cnt_total_out_nodes)<neuron_array_size)
            REG_neuron_array_end<=var_layer_out_nodes-REG_cnt_total_out_nodes-1;
          else
            REG_neuron_array_end<=neuron_array_size-1;
        end
        STATE_PROCESS_1 : 
        begin
          if (ram_1_addr<=REG_ram_1_read_beg + var_layer_in_nodes[ram_1_addr_size-1:0]-1)
          begin
            ram_1_addr<=ram_1_addr+1'b1;       
          end
          REG_data_in[0]<=ram_1_read;
          for(i=1;i<=REG_neuron_array_end;i=i+1)    
          begin
            REG_data_in[i]<=REG_data_in[i-1];
          end
          neuron_enable[0]<=1;                  
          for(i=1;i<=REG_neuron_array_end;i=i+1) 
          begin
            neuron_enable[i]<=neuron_enable[i-1];
          end
        end
        STATE_PROCESS_2 : 
        begin
          REG_cnt_PROCESS_2 <= REG_cnt_PROCESS_2+1;
          ram_1_addr      <=REG_ram_1_write_end;
          REG_data_in[0]  <=0;                    
          neuron_enable[0]<=0;                    
          for(i=1;i<=REG_neuron_array_end;i=i+1)
          begin
            REG_data_in[i]  <=REG_data_in[i-1];
            neuron_enable[i]<=neuron_enable[i-1]; 
          end 
          for(i=0;i<=REG_neuron_array_end;i=i+1) 
          begin
            if (~neuron_enable[i])
            begin
              if (~REG_RL_enable)
                REG_data_out[i]<=sig_output_data[i];
              else
                REG_data_temp[i]<=sig_output_data[i];
            end
          end
          // for(i=REG_neuron_array_end;i<=neuron_array_size-1;i=i+1) 
          // begin
          //     REG_data_out[i]<=0;
          // end
        end
        STATE_WRITE :
        begin
          REG_cnt_out_nodes   <=REG_cnt_out_nodes+1'b1; 
          ram_1_addr          <=ram_1_addr+1'b1; 
          REG_data_out[REG_neuron_array_end] <=0; // find this bug Zheng Wang 
          for(i=1;i<=REG_neuron_array_end;i=i+1) 
          begin
            REG_data_out[i-1] <=REG_data_out[i];
          end
        end
        STATE_WRITE_DONE :
        begin
          REG_cnt_out_nodes<=0; 
          REG_cnt_total_out_nodes<=REG_cnt_total_out_nodes+neuron_array_size; 
          REG_ram_1_write_end<=ram_1_addr;   // store 1 after real write data end
          ram_1_addr<=REG_ram_1_read_beg; 
        end
        STATE_BRANCH : // backup REG_ram_1_read_beg to cfg mem <bandwidth issue, reserved state>
        begin
        //   ram_cfg_addr<=ram_cfg_addr-1'b1;
        //   ram_cfg_write<=REG_ram_1_read_beg;
        end
        STATE_BRANCH_DONE :  // backup REG_ram_1_write_beg to cfg mem <bandwidth issue, reserved state>
        begin
        //   ram_cfg_addr<=ram_cfg_addr+1'b1;
        //   ram_cfg_write<=REG_ram_1_write_beg;
          REG_cnt_total_out_nodes<=0;  // clear this flag for next layer
        end
        STATE_UPDATE_INST :  // backup REG_ram_1_write_beg to cfg mem <bandwidth issue, reserved state>
        begin
        //  if (~var_layer_done)
        //  begin
        //    ram_cfg_addr<=ram_cfg_addr+1'b1;
        //    ram_cfg_write<=0;
        //  end
          if (var_layer_done)
            ram_1_addr<=REG_ram_1_write_beg;
          else
            REG_cnt_layer<=REG_cnt_layer+1;
        end
        STATE_DATA_END :
        begin
          ram_1_addr<=ram_1_addr+1;
        end
        STATE_OUTPUT :    
	begin
          ram_1_addr<=ram_1_addr+1;
          out_addr<=out_addr+1;
	end
        STATE_OUTPUT_DONE :    
	begin
          out_data_received<=1;
          REG_cnt_layer<=0;
	end
        STATE_RL : 
	begin
          REG_RL_enable<=1;
          ram_act_addr<=ram_act_addr+1;
	end
        STATE_ERROR : 
	begin
          out_error<=1;
	end
        default: ;
      endcase
    end
  end

  always @ (*)        //combine to above block can simplify the testbench//Set it out of the FSM is OK
  begin: FSM_state_output_ctrl
    integer i;
    web_ram_1=1;    //avoid latch
    csb_ram_1=1;    
    csb_ram_cfg = 1;
    web_ram_cfg = 1;
    csb_ram_act = 1;
    web_ram_act = 1;
    csb_ram_rewa = 1;
    web_ram_rewa = 1;
    sig_config_done_weight =0;
    sig_config_done_cfg    =0;             //cfg_ram compeleted flag
    sig_config_done_act    =0;
    sig_config_done_rewa   =0;
    sig_config_done_data   =0;             //data_ram compeleted flag
    sig_size_in_out_total  =0;
    sig_error=0;
    sig_ram_1_wrap=0;
    out_data=0;
    out_wea_data=0;
    for(i=0;i<=neuron_array_size-1;i=i+1) 
    begin
        sig_weight_in[i]=0;                //void latch
        sig_weight_wr[i]=0;                //memory write enable 
        sig_neuron_clear[i]=0;
    end
    case(state_ctrl)
      STATE_RST :
      begin
      end
      STATE_IDLE :
      begin
        if (in_weight_en)
        begin
          sig_weight_in[REG_cnt_class]=in_CFG;
          sig_weight_wr[REG_cnt_class]=1;         //memory write enable 
        end
      end
      CONFIG_CFG_RAM : 
      begin
        csb_ram_cfg = 0;
        web_ram_cfg = 0;
        if (ram_cfg_addr >= cfg_ram_size-1)
        begin
          sig_config_done_cfg = 1;
        end
      end
      CONFIG_WEIGHT_RAM :                              
      begin
        sig_weight_in[REG_cnt_class]=in_CFG;
        sig_weight_wr[REG_cnt_class]=1;         //memory write enable 
        if (REG_cnt_class >= neuron_array_size) //REG_cnt_class should be 8bits wide
        begin
          sig_config_done_weight=1;
        end
      end
      CONFIG_ACTION_RAM : 
      begin
        csb_ram_act = 0;
        web_ram_act = 0;
        if (ram_act_addr >= act_ram_size-1)
        begin
          sig_config_done_act = 1;
        end
      end
      CONFIG_REWARD_RAM : 
      begin
        csb_ram_rewa = 0;
        web_ram_rewa = 0;
        if (ram_rewa_addr >= rewa_ram_size-1)
        begin
          sig_config_done_rewa = 1;
        end
      end
      STATE_LOAD_INST : 
      begin
        csb_ram_cfg = 0;
        web_ram_cfg = 1;
      end
      STATE_DECODE_INST :    // decide if ram_1 is full or write data needs to be rapped around
      begin
        sig_size_in_out_total = var_layer_in_nodes + var_layer_out_nodes;
        if (sig_size_in_out_total>ram_1_size) sig_error=1;
        if (var_layer_in_nodes==0 || var_layer_out_nodes==0) sig_error=1;
        if ((REG_ram_1_write_end + var_layer_out_nodes)>ram_1_size)
        begin 
          if (var_layer_in_nodes<REG_ram_1_write_beg) sig_ram_1_wrap=1;
          else sig_error=1;
        end
      end
      STATE_LOAD_DATA :
      begin
        web_ram_1=0;
        csb_ram_1=0;
        if (ram_1_addr >= var_layer_in_nodes[ram_1_addr_size-1:0]-1)
        begin
          sig_config_done_data = 1;
        end
      end
      STATE_PROCESS_0: 
      begin
        web_ram_1=1;
        csb_ram_1=0;
      end
      STATE_PROCESS_1: 
      begin
        web_ram_1=1;
        csb_ram_1=0;
      end
      STATE_WRITE :
      begin
        web_ram_1=0;
        csb_ram_1=0;
      end
      STATE_WRITE_DONE :
      begin
        for(i=0;i<=neuron_array_size-1;i=i+1) 
        begin
          sig_neuron_clear[i]=1;
        end
      end
      STATE_BRANCH_DONE :
      begin
      //  csb_ram_cfg=0;
      //  web_ram_cfg=0;
      end
      STATE_UPDATE_INST :
      begin
      //  csb_ram_cfg=0;
      //  web_ram_cfg=0;
      end
      STATE_DATA_END : 
      begin
        web_ram_1=1;
        csb_ram_1=0;
      end
      STATE_OUTPUT : 
      begin
        web_ram_1=1;
        csb_ram_1=0;
        out_data=ram_1_read;
	out_wea_data=1;
      end
      STATE_OUTPUT_DONE :
      begin
        web_ram_1=1;
        csb_ram_1=0;
        out_data=ram_1_read;
	out_wea_data=1;
      end
      STATE_RL : 
      begin
        web_ram_act=1;
        csb_ram_act=0;
      end
      default: ;
    endcase
  end

  always @ (posedge clk)
  begin
    ram_1_write<=in_data|REG_data_out[0];
  end
  
  assign out_cfg_done=sig_config_done_weight|sig_config_done_cfg|sig_config_done_data;

//----------2. Instantiation of neurons---------------------
  genvar k;
  generate
    for(k=0;k<neuron_array_size;k=k+1)
    begin:Neuron_gen_loop
      neuron ARC_neuron(                     // neuron name should be changing with k
         .out_data     (sig_output_data[k])
        ,.in_weight    (sig_weight_in[k]) 
        ,.in_neuron_clear  (sig_neuron_clear[k])
        ,.in_data_n    (REG_data_in[k])      //Distinguish it from NN_input "in_data"
        ,.mode         (mode)
        ,.threshold    (threshold)
        ,.enable       (neuron_enable[k])    //some association must connect with the in_data_en signals/////neuron_enable<=1;
        ,.in_weight_wr (sig_weight_wr[k])
        ,.rst          (rst) 
        ,.clk          (clk)
      ); 
    end
  endgenerate
  
//  assign out_parity = REG_config_chain[cfg_reg_size-1]^REG_config_chain[cfg_reg_size-2]^REG_config_chain[cfg_reg_size-3]^REG_config_chain[cfg_reg_size-4]^REG_config_chain[cfg_reg_size-5]^REG_config_chain[cfg_reg_size-6]^REG_config_chain[cfg_reg_size-7]^REG_config_chain[cfg_reg_size-8];

//----------3. Instantiation of Memory---------------------
`ifdef use_xilinx_ip
  ram_1 MEM_STAGE_DATA(                      //simple single port mem
       .clka    (clk)                        //~clk
      ,.ena     (~csb_ram_1)
      ,.wea     (~web_ram_1)
      ,.addra   (ram_1_addr)
      ,.dina    (ram_1_write)
      ,.douta   (ram_1_read)
  );

  ram_1 CONFIG_RAM_CFG(                   // 1 K ram for config parameter
       .clka    (clk)                        // clk
      ,.ena     (~csb_ram_cfg)
      ,.wea     (~web_ram_cfg)
      ,.addra   (ram_cfg_addr)
      ,.dina    (ram_cfg_write)
      ,.douta   (ram_cfg_read)
  );

  ram_1 RL_ACTION_RAM(                   // 1 K ram for config parameter
       .clka    (clk)                        // clk
      ,.ena     (~csb_ram_act)
      ,.wea     (~web_ram_act)
      ,.addra   (ram_act_addr)
      ,.dina    (ram_act_write)
      ,.douta   (ram_act_read)
  );

  ram_1 RL_REWARD_RAM(                   // 1 K ram for config parameter
       .clka    (clk)                        // clk
      ,.ena     (~csb_ram_rewa)
      ,.wea     (~web_ram_rewa)
      ,.addra   (ram_rewa_addr)
      ,.dina    (ram_rewa_write)
      ,.douta   (ram_rewa_read)
  );
`else
`endif

endmodule
