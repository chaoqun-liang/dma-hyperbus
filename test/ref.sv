`timescale 1 ns/1 ps

`include "axi/typedef.svh"
`include "axi/assign.svh"
`include "register_interface/typedef.svh"
`include "register_interface/assign.svh"

module xbar_hyperbus_tb;

  // ------------------------------------------------------------
  // Parameters
  // ------------------------------------------------------------
  parameter int unsigned NumChips        = 2;
  parameter int unsigned NumPhys         = 2;
  parameter int unsigned NumMasters      = 2;    // Number of AXI masters
  parameter int unsigned NumSlaves       = 1;    // Just hyperbus
  
  // XBAR Configuration
  parameter int unsigned AxiIdWidthMasters = 6;
  parameter int unsigned AxiIdUsed         = 3;
  parameter int unsigned AxiDataWidth      = 64;
  parameter int unsigned AxiAddrWidth      = 32;
  parameter int unsigned AxiUserWidth      = 1;
  parameter int unsigned MaxMstTrans       = 8;
  parameter int unsigned MaxSlvTrans       = 8;
  
  // Clock parameters
  parameter time TbCyclTime = 5ns;
  parameter time TbApplTime = 2ns;
  parameter time TbTestTime = 8ns;
  
  // Test parameters
  parameter int unsigned TbNumWrites = 100;
  parameter int unsigned TbNumReads  = 100;
  
  // ------------------------------------------------------------
  // Types
  // ------------------------------------------------------------
  typedef logic [AxiIdWidthMasters-1:0] id_mst_t;
  typedef logic [AxiAddrWidth-1:0]      addr_t;
  typedef logic [AxiDataWidth-1:0]      data_t;
  typedef logic [AxiDataWidth/8-1:0]    strb_t;
  typedef logic [AxiUserWidth-1:0]      user_t;
  typedef axi_pkg::xbar_rule_32_t       rule_t;
  
  `AXI_TYPEDEF_AW_CHAN_T(aw_chan_t, addr_t, id_mst_t, user_t)
  `AXI_TYPEDEF_W_CHAN_T(w_chan_t, data_t, strb_t, user_t)
  `AXI_TYPEDEF_B_CHAN_T(b_chan_t, id_mst_t, user_t)
  `AXI_TYPEDEF_AR_CHAN_T(ar_chan_t, addr_t, id_mst_t, user_t)
  `AXI_TYPEDEF_R_CHAN_T(r_chan_t, data_t, id_mst_t, user_t)
  `AXI_TYPEDEF_REQ_T(req_t, aw_chan_t, w_chan_t, ar_chan_t)
  `AXI_TYPEDEF_RESP_T(resp_t, b_chan_t, r_chan_t)
  
  // ------------------------------------------------------------
  // XBAR Configuration
  // ------------------------------------------------------------
  localparam axi_pkg::xbar_cfg_t xbar_cfg = '{
    NoSlvPorts:         NumMasters,
    NoMstPorts:         NumSlaves,
    MaxMstTrans:        MaxMstTrans,
    MaxSlvTrans:        MaxSlvTrans,
    FallThrough:        1'b0,
    LatencyMode:        axi_pkg::CUT_ALL_AX,
    PipelineStages:     1,
    AxiIdWidthSlvPorts: AxiIdWidthMasters,
    AxiIdUsedSlvPorts:  AxiIdUsed,
    UniqueIds:          1'b0,
    AxiAddrWidth:       AxiAddrWidth,
    AxiDataWidth:       AxiDataWidth,
    NoAddrRules:        NumSlaves
  };
  
  // Address map: HyperBus memory region
  localparam rule_t [xbar_cfg.NoAddrRules-1:0] AddrMap = '{
    '{
      idx:        0,
      start_addr: 32'h8000_0000,  // HyperBus region start
      end_addr:   32'h9000_0000,  // HyperBus region end (256MB)
      default:    '0
    }
  };
  
  // ------------------------------------------------------------
  // Clock and Reset
  // ------------------------------------------------------------
  logic clk, rst_n, end_of_sim;
  
  clk_rst_gen #(
    .ClkPeriod    ( TbCyclTime ),
    .RstClkCycles ( 5 )
  ) i_clk_gen (
    .clk_o  ( clk   ),
    .rst_no ( rst_n )
  );
  
  // ------------------------------------------------------------
  // AXI Interfaces
  // ------------------------------------------------------------
  // Master interfaces (from random masters to XBAR slave ports)
  AXI_BUS_DV #(
    .AXI_ADDR_WIDTH ( AxiAddrWidth ),
    .AXI_DATA_WIDTH ( AxiDataWidth ),
    .AXI_ID_WIDTH   ( AxiIdWidthMasters ),
    .AXI_USER_WIDTH ( AxiUserWidth )
  ) master_dv [NumMasters-1:0] (clk);
  
  // Slave interfaces (from XBAR master ports to HyperBus)
  AXI_BUS #(
    .AXI_ADDR_WIDTH ( AxiAddrWidth ),
    .AXI_DATA_WIDTH ( AxiDataWidth ),
    .AXI_ID_WIDTH   ( AxiIdWidthMasters ),
    .AXI_USER_WIDTH ( AxiUserWidth )
  ) slave [NumSlaves-1:0] ();
  
  // ------------------------------------------------------------
  // Random Masters (Will be replaced by DMA later)
  // ------------------------------------------------------------
  typedef axi_test::axi_rand_master #(
    .AW                   ( AxiAddrWidth ),
    .DW                   ( AxiDataWidth ),
    .IW                   ( AxiIdWidthMasters ),
    .UW                   ( AxiUserWidth ),
    .TA                   ( TbApplTime ),
    .TT                   ( TbTestTime ),
    .MAX_READ_TXNS        ( 8 ),
    .MAX_WRITE_TXNS       ( 8 ),
    .AX_MIN_WAIT_CYCLES   ( 0 ),
    .AX_MAX_WAIT_CYCLES   ( 5 ),
    .W_MIN_WAIT_CYCLES    ( 0 ),
    .W_MAX_WAIT_CYCLES    ( 3 ),
    .RESP_MIN_WAIT_CYCLES ( 0 ),
    .RESP_MAX_WAIT_CYCLES ( 5 ),
    .AXI_BURST_FIXED      ( 1'b0 ),
    .AXI_BURST_INCR       ( 1'b1 ),
    .AXI_BURST_WRAP       ( 1'b0 ),
    .AXI_ATOPS            ( 1'b0 )
  ) axi_rand_master_t;
  
  logic [NumMasters-1:0] master_end_of_sim;
  
  generate
    for (genvar i = 0; i < NumMasters; i++) begin : gen_rand_masters
      initial begin
        automatic axi_rand_master_t rand_master = new(master_dv[i]);
        master_end_of_sim[i] <= 1'b0;
        
        // Configure memory region for this master
        // All masters target the HyperBus memory region
        rand_master.add_memory_region(
          AddrMap[0].start_addr,  // Start: 0x8000_0000
          AddrMap[0].end_addr,    // End:   0x9000_0000
          axi_pkg::DEVICE_NONBUFFERABLE
        );
        
        rand_master.reset();
        @(posedge rst_n);
        
        $display("Time %0t: Master %0d starting %0d reads and %0d writes", 
                 $time, i, TbNumReads, TbNumWrites);
        
        // Start random traffic
        rand_master.run(TbNumReads, TbNumWrites);
        
        master_end_of_sim[i] <= 1'b1;
        $display("Time %0t: Master %0d finished", $time, i);
      end
    end
  endgenerate
  
  // ------------------------------------------------------------
  // XBAR Instance
  // ------------------------------------------------------------
  axi_xbar_intf #(
    .AXI_USER_WIDTH ( AxiUserWidth ),
    .Cfg            ( xbar_cfg ),
    .rule_t         ( rule_t )
  ) i_xbar (
    .clk_i                  ( clk ),
    .rst_ni                 ( rst_n ),
    .test_i                 ( 1'b0 ),
    .slv_ports              ( master_dv ),   // Connect to random masters
    .mst_ports              ( slave ),       // Connect to HyperBus
    .addr_map_i             ( AddrMap ),
    .en_default_mst_port_i  ( '0 ),
    .default_mst_port_i     ( '0 )
  );
  
  // ------------------------------------------------------------
  // HyperBus Instance (using your existing dut_if)
  // ------------------------------------------------------------
  logic hyperbus_end_sim;
  
  // Register interface for hyperbus configuration
  REG_BUS #(
    .ADDR_WIDTH ( 8 ),
    .DATA_WIDTH ( 32 )
  ) reg_bus (.clk_i(clk));
  
  // Connect XBAR master port 0 to hyperbus
  dut_if #(
    .TbTestTime      ( TbTestTime ),
    .AxiDataWidth    ( AxiDataWidth ),
    .AxiAddrWidth    ( AxiAddrWidth ),
    .AxiIdWidth      ( AxiIdWidthMasters ),
    .AxiUserWidth    ( AxiUserWidth ),
    .RegAw           ( 8 ),
    .RegDw           ( 32 ),
    .NumChips        ( NumChips ),
    .NumPhys         ( NumPhys ),
    .IsClockODelayed ( 0 ),
    .axi_rule_t      ( rule_t )
  ) i_hyperbus (
    .clk_i      ( clk ),
    .rst_ni     ( rst_n ),
    .end_sim_i  ( hyperbus_end_sim ),
    .axi_slv_if ( slave[0] ),  // Connect to XBAR's master port 0
    .reg_slv_if ( reg_bus )
  );
  
  // ------------------------------------------------------------
  // Register Driver for HyperBus Configuration
  // ------------------------------------------------------------
  reg_test::reg_driver #(
    .AW ( 8 ),
    .DW ( 32 ),
    .TA ( TbApplTime ),
    .TT ( TbTestTime )
  ) i_reg_master = new( reg_bus );
  
  // ------------------------------------------------------------
  // Simple Traffic Monitor (Optional)
  // ------------------------------------------------------------
  initial begin : traffic_monitor
    @(posedge rst_n);
    
    $display("==========================================");
    $display("Traffic Monitor Started");
    $display("Masters: %0d, XBAR Ports: %0d", NumMasters, NumSlaves);
    $display("Address Range: 0x%h to 0x%h", 
             AddrMap[0].start_addr, AddrMap[0].end_addr);
    $display("==========================================");
    
    // Simple transaction counter
    int aw_count[NumMasters] = '{default: 0};
    int ar_count[NumMasters] = '{default: 0};
    
    forever begin
      @(posedge clk);
      
      // Count AW transactions per master
      for (int i = 0; i < NumMasters; i++) begin
        if (master_dv[i].aw_valid && master_dv[i].aw_ready) begin
          aw_count[i]++;
          if (aw_count[i] % 50 == 0) begin
            $display("Time %0t: Master %0d AW #%0d to addr 0x%h",
                     $time, i, aw_count[i], master_dv[i].aw_addr);
          end
        end
        
        // Count AR transactions per master
        if (master_dv[i].ar_valid && master_dv[i].ar_ready) begin
          ar_count[i]++;
          if (ar_count[i] % 50 == 0) begin
            $display("Time %0t: Master %0d AR #%0d to addr 0x%h",
                     $time, i, ar_count[i], master_dv[i].ar_addr);
          end
        end
      end
    end
  end
  
  // ------------------------------------------------------------
  // Test Control
  // ------------------------------------------------------------
  initial begin : proc_test_control
    logic reg_error;
    
    // Initialize
    i_reg_master.reset_master();
    end_of_sim = 1'b0;
    hyperbus_end_sim = 1'b0;
    
    // Wait for reset
    @(posedge rst_n);
    #100ns;
    
    // Configure HyperBus registers
    $display("Time %0t: Configuring HyperBus registers...", $time);
    
    // Set initial latency to additional latency
    i_reg_master.send_write('h4, 'h1, '1, reg_error);
    if (reg_error) $error("Register write 0x4 failed");
    
    // Reduce max burst length from 350 to 250
    i_reg_master.send_write('h8, 'd250, '1, reg_error);
    if (reg_error) $error("Register write 0x8 failed");
    
    $display("Time %0t: HyperBus configuration complete", $time);
    
    // Wait for all masters to finish
    $display("Time %0t: Waiting for %0d masters to complete...", 
             $time, NumMasters);
    
    wait (&master_end_of_sim);
    
    // Add delay for any outstanding transactions
    #2000ns;
    
    $display("==========================================");
    $display("Time %0t: TEST COMPLETE", $time);
    $display("All %0d masters finished", NumMasters);
    $display("==========================================");
    
    end_of_sim = 1'b1;
    hyperbus_end_sim = 1'b1;
    #100ns;
    $finish();
  end
  
  // ------------------------------------------------------------
  // AXI Channel Logger for Debug (Optional)
  // ------------------------------------------------------------
  // Log traffic on XBAR master port 0 (to HyperBus)
  axi_chan_logger #(
    .TestTime   ( TbTestTime ),
    .LoggerName ( "xbar_to_hyperbus" ),
    .aw_chan_t  ( aw_chan_t ),
    .w_chan_t   ( w_chan_t ),
    .b_chan_t   ( b_chan_t ),
    .ar_chan_t  ( ar_chan_t ),
    .r_chan_t   ( r_chan_t )
  ) i_xbar_logger (
    .clk_i      ( clk ),
    .rst_ni     ( rst_n ),
    .end_sim_i  ( end_of_sim ),
    // Monitor XBAR output to HyperBus
    .aw_chan_i  ( slave[0].aw ),
    .aw_valid_i ( slave[0].aw_valid ),
    .aw_ready_i ( slave[0].aw_ready ),
    .w_chan_i   ( slave[0].w ),
    .w_valid_i  ( slave[0].w_valid ),
    .w_ready_i  ( slave[0].w_ready ),
    .b_chan_i   ( slave[0].b ),
    .b_valid_i  ( slave[0].b_valid ),
    .b_ready_i  ( slave[0].b_ready ),
    .ar_chan_i  ( slave[0].ar ),
    .ar_valid_i ( slave[0].ar_valid ),
    .ar_ready_i ( slave[0].ar_ready ),
    .r_chan_i   ( slave[0].r ),
    .r_valid_i  ( slave[0].r_valid ),
    .r_ready_i  ( slave[0].r_ready )
  );
  
endmodule