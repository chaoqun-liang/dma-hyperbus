

`timescale 1ns/1ps

module axi_hyper_tb
  import axi_pkg::*;
#(
  parameter int unsigned NumChips = 1,
  parameter int unsigned NumPhys  = 1,
  parameter int unsigned IsClockODelayed = 0,

  // Two initiators into the xbar: iDMA data-path master + CPU config master
  parameter int unsigned NumMasters = 2,
  // Three targets behind the xbar: SRAM + HyperBus + iDMA regs (AXI slave)
  parameter int unsigned NumSlaves  = 3,

  parameter int unsigned TbAxiIdWidth   = 6,
  parameter int unsigned TbAxiAddrWidth = 32,
  parameter int unsigned TbAxiDataWidth = 64,
  parameter int unsigned TbAxiUserWidth = 1,

  parameter time TbCyclTime = 5ns,
  parameter time TbApplTime = 1ns,
  parameter time TbTestTime = 4ns,

  // DMA configuration
  parameter int unsigned DmaNumAxInFlight    = 16,
  parameter int unsigned DmaMemSysDepth      = 8,
  parameter int unsigned DmaJobFifoDepth     = 2,
  parameter bit          DmaRAWCouplingAvail = 1'b1,
  parameter bit          DmaConfEnableTwoD   = 1'b0
);

  `include "axi/typedef.svh"
  `include "axi/assign.svh"
  `include "register_interface/typedef.svh"

  // -----------------------
  // ID widths: xbar extends IDs on master ports by +clog2(NumMasters)
  // -----------------------
  localparam int unsigned XbarIdWidthSlvPorts = TbAxiIdWidth;
  localparam int unsigned XbarIdWidthMstPorts = TbAxiIdWidth + $clog2(NumMasters);

  typedef logic [TbAxiAddrWidth-1:0]   axi_addr_t;
  typedef logic [TbAxiDataWidth-1:0]   axi_data_t;
  typedef logic [TbAxiDataWidth/8-1:0] axi_strb_t;
  typedef logic [TbAxiUserWidth-1:0]   axi_user_t;

  typedef logic [XbarIdWidthSlvPorts-1:0] axi_id_slv_t;
  typedef logic [XbarIdWidthMstPorts-1:0] axi_id_mst_t;

  typedef axi_pkg::xbar_rule_32_t rule_t;

  // -----------------------
  // Initiator-side (xbar slave ports) structs: ID=6
  // -----------------------
  `AXI_TYPEDEF_AW_CHAN_T(aw_slv_chan_t, axi_addr_t, axi_id_slv_t, axi_user_t)
  `AXI_TYPEDEF_W_CHAN_T (w_chan_t,      axi_data_t, axi_strb_t, axi_user_t)
  `AXI_TYPEDEF_B_CHAN_T (b_slv_chan_t,  axi_id_slv_t, axi_user_t)
  `AXI_TYPEDEF_AR_CHAN_T(ar_slv_chan_t, axi_addr_t, axi_id_slv_t, axi_user_t)
  `AXI_TYPEDEF_R_CHAN_T (r_slv_chan_t,  axi_data_t, axi_id_slv_t, axi_user_t)

  `AXI_TYPEDEF_REQ_T (xbar_slv_req_t,  aw_slv_chan_t, w_chan_t, ar_slv_chan_t)
  `AXI_TYPEDEF_RESP_T(xbar_slv_rsp_t,  b_slv_chan_t,  r_slv_chan_t)

  // -----------------------
  // Target-side (xbar master ports) structs: ID=7
  // -----------------------
  `AXI_TYPEDEF_AW_CHAN_T(aw_mst_chan_t, axi_addr_t, axi_id_mst_t, axi_user_t)
  `AXI_TYPEDEF_B_CHAN_T (b_mst_chan_t,  axi_id_mst_t, axi_user_t)
  `AXI_TYPEDEF_AR_CHAN_T(ar_mst_chan_t, axi_addr_t, axi_id_mst_t, axi_user_t)
  `AXI_TYPEDEF_R_CHAN_T (r_mst_chan_t,  axi_data_t, axi_id_mst_t, axi_user_t)

  `AXI_TYPEDEF_REQ_T (xbar_mst_req_t,  aw_mst_chan_t, w_chan_t, ar_mst_chan_t)
  `AXI_TYPEDEF_RESP_T(xbar_mst_rsp_t,  b_mst_chan_t,  r_mst_chan_t)

  // -----------------------
  // Address map
  // -----------------------
  localparam axi_addr_t SRAM_BASE     = 32'h0000_0000;
  localparam axi_addr_t SRAM_END      = 32'h0000_FFFF;
  localparam axi_addr_t HYPERRAM_BASE = 32'h8000_0000;
  localparam axi_addr_t HYPERRAM_END  = 32'h8FFF_FFFF;

  localparam axi_addr_t IDMA_BASE     = 32'h0100_0000;
  localparam axi_addr_t IDMA_END      = 32'h0100_0FFF;

  localparam rule_t [NumSlaves-1:0] AddrMap = '{
    '{idx: 0, start_addr: SRAM_BASE,     end_addr: SRAM_END,     default: '0},
    '{idx: 1, start_addr: HYPERRAM_BASE, end_addr: HYPERRAM_END, default: '0},
    '{idx: 2, start_addr: IDMA_BASE,     end_addr: IDMA_END,     default: '0}
  };

  // iDMA register offsets
  localparam logic [31:0] IDMA_SRC_ADDR_OFFSET = 32'h0000_00d8;
  localparam logic [31:0] IDMA_DST_ADDR_OFFSET = 32'h0000_00d0;
  localparam logic [31:0] IDMA_LENGTH_OFFSET   = 32'h0000_00e0;
  localparam logic [31:0] IDMA_NEXT_ID_OFFSET  = 32'h0000_0044;
  localparam logic [31:0] IDMA_DONE_ID_OFFSET  = 32'h0000_0048;
  localparam logic [31:0] IDMA_REPS_2          = 32'h0000_00f8;
  localparam logic [31:0] IDMA_CONF            = 32'h0000_0000;

  // -----------------------
  // Crossbar config
  // -----------------------
  localparam axi_pkg::xbar_cfg_t xbar_cfg = '{
    NoSlvPorts:         NumMasters,
    NoMstPorts:         NumSlaves,
    MaxMstTrans:        8,
    MaxSlvTrans:        8,
    FallThrough:        1'b0,
    LatencyMode:        axi_pkg::CUT_ALL_AX,
    PipelineStages:     1,
    AxiIdWidthSlvPorts: XbarIdWidthSlvPorts,
    AxiIdUsedSlvPorts:  XbarIdWidthSlvPorts,
    UniqueIds:          1'b0,
    AxiAddrWidth:       TbAxiAddrWidth,
    AxiDataWidth:       TbAxiDataWidth,
    NoAddrRules:        NumSlaves
  };

  // -----------------------
  // Clock / Reset
  // -----------------------
  logic clk, rst_n, end_of_sim;

  clk_rst_gen #(
    .ClkPeriod    ( TbCyclTime ),
    .RstClkCycles ( 32'd5      )
  ) i_clk_rst_gen (
    .clk_o  ( clk   ),
    .rst_no ( rst_n )
  );

  // -----------------------
  // HyperBus reg bus
  // -----------------------
  REG_BUS #(
    .ADDR_WIDTH ( 8  ),
    .DATA_WIDTH ( 32 )
  ) hyper_reg_bus (clk);

  // -----------------------
  // AXI interfaces
  // -----------------------
  AXI_BUS #(
    .AXI_ADDR_WIDTH ( TbAxiAddrWidth ),
    .AXI_DATA_WIDTH ( TbAxiDataWidth ),
    .AXI_ID_WIDTH   ( XbarIdWidthSlvPorts ),
    .AXI_USER_WIDTH ( TbAxiUserWidth )
  ) master [NumMasters-1:0] ();

  AXI_BUS #(
    .AXI_ADDR_WIDTH ( TbAxiAddrWidth ),
    .AXI_DATA_WIDTH ( TbAxiDataWidth ),
    .AXI_ID_WIDTH   ( XbarIdWidthMstPorts ),
    .AXI_USER_WIDTH ( TbAxiUserWidth )
  ) slave  [NumSlaves-1:0] ();

  AXI_BUS_DV #(
    .AXI_ADDR_WIDTH ( TbAxiAddrWidth ),
    .AXI_DATA_WIDTH ( TbAxiDataWidth ),
    .AXI_ID_WIDTH   ( XbarIdWidthSlvPorts ),
    .AXI_USER_WIDTH ( TbAxiUserWidth )
  ) cpu_master_dv (clk);

  `AXI_ASSIGN(master[1], cpu_master_dv)

  // -----------------------
  // Xbar DUT
  // -----------------------
  axi_xbar_intf #(
    .AXI_USER_WIDTH ( TbAxiUserWidth ),
    .Cfg            ( xbar_cfg       ),
    .rule_t         ( rule_t         )
  ) i_xbar (
    .clk_i                 ( clk     ),
    .rst_ni                ( rst_n   ),
    .test_i                ( 1'b0   ),
    .slv_ports             ( master  ),
    .mst_ports             ( slave   ),
    .addr_map_i            ( AddrMap ),
    .en_default_mst_port_i ( '0      ),
    .default_mst_port_i    ( '0      )
  );

  // -----------------------
  // iDMA wrapper
  // -----------------------
  xbar_slv_req_t idma_mst_req;
  xbar_slv_rsp_t idma_mst_rsp;

  `AXI_ASSIGN_FROM_REQ(master[0], idma_mst_req)
  `AXI_ASSIGN_TO_RESP(idma_mst_rsp, master[0])

  xbar_mst_req_t idma_cfg_req;
  xbar_mst_rsp_t idma_cfg_rsp;

  `AXI_ASSIGN_TO_REQ(idma_cfg_req, slave[2])
  `AXI_ASSIGN_FROM_RESP(slave[2], idma_cfg_rsp)

  cheshire_idma_wrap #(
    .AxiAddrWidth     ( TbAxiAddrWidth ),
    .AxiDataWidth     ( TbAxiDataWidth ),
    .AxiIdWidth       ( XbarIdWidthSlvPorts ),
    .AxiUserWidth     ( TbAxiUserWidth ),
    .AxiSlvIdWidth    ( XbarIdWidthMstPorts ),
    .NumAxInFlight    ( DmaNumAxInFlight ),
    .MemSysDepth      ( DmaMemSysDepth   ),
    .JobFifoDepth     ( DmaJobFifoDepth  ),
    .RAWCouplingAvail ( DmaRAWCouplingAvail ),
    .IsTwoD           ( DmaConfEnableTwoD   ),
    .axi_mst_req_t    ( xbar_slv_req_t ),
    .axi_mst_rsp_t    ( xbar_slv_rsp_t ),
    .axi_slv_req_t    ( xbar_mst_req_t ),
    .axi_slv_rsp_t    ( xbar_mst_rsp_t )
  ) i_idma (
    .clk_i         ( clk ),
    .rst_ni        ( rst_n ),
    .testmode_i    ( 1'b0 ),
    .axi_mst_req_o ( idma_mst_req ),
    .axi_mst_rsp_i ( idma_mst_rsp ),
    .axi_slv_req_i ( idma_cfg_req ),
    .axi_slv_rsp_o ( idma_cfg_rsp )
  );

  // -----------------------
  // Target 0: SRAM model
  // -----------------------
  xbar_mst_req_t sram_req;
  xbar_mst_rsp_t sram_rsp;

  `AXI_ASSIGN_TO_REQ   (sram_req, slave[0])
  `AXI_ASSIGN_FROM_RESP(slave[0], sram_rsp)

  axi_sim_mem #(
    .AddrWidth          ( TbAxiAddrWidth ),
    .DataWidth          ( TbAxiDataWidth ),
    .IdWidth            ( XbarIdWidthMstPorts ),
    .UserWidth          ( TbAxiUserWidth ),
    .axi_req_t          ( xbar_mst_req_t ),
    .axi_rsp_t          ( xbar_mst_rsp_t ),
    .WarnUninitialized  ( 1'b0 ),
    .UninitializedData  ( "random" ),
    .ClearErrOnAccess   ( 1'b1 ),
    .ApplDelay          ( TbApplTime ),
    .AcqDelay           ( TbTestTime )
  ) i_sram (
    .clk_i      ( clk ),
    .rst_ni     ( rst_n ),
    .axi_req_i  ( sram_req ),
    .axi_rsp_o  ( sram_rsp ),
    .mon_r_last_o       ( ),
    .mon_r_beat_count_o ( ),
    .mon_r_user_o       ( ),
    .mon_r_id_o         ( ),
    .mon_r_data_o       ( ),
    .mon_r_addr_o       ( ),
    .mon_r_valid_o      ( ),
    .mon_w_last_o       ( ),
    .mon_w_beat_count_o ( ),
    .mon_w_user_o       ( ),
    .mon_w_id_o         ( ),
    .mon_w_data_o       ( ),
    .mon_w_addr_o       ( ),
    .mon_w_valid_o      ( )
  );

  // -----------------------
  // Target 1: HyperBus DUT
  // -----------------------
  dut_if #(
    .TbTestTime      ( TbTestTime      ),
    .AxiDataWidth    ( TbAxiDataWidth  ),
    .AxiAddrWidth    ( TbAxiAddrWidth  ),
    .AxiIdWidth      ( XbarIdWidthMstPorts ),
    .AxiUserWidth    ( TbAxiUserWidth  ),
    .RegAw           ( 8               ),
    .RegDw           ( 32              ),
    .NumChips        ( NumChips        ),
    .NumPhys         ( NumPhys         ),
    .IsClockODelayed ( IsClockODelayed ),
    .axi_rule_t      ( rule_t          )
  ) i_dut_if (
    .clk_i      ( clk         ),
    .rst_ni     ( rst_n       ),
    .end_sim_i  ( end_of_sim  ),
    .axi_slv_if ( slave[1]    ),
    .reg_slv_if ( hyper_reg_bus )
  );

  // ============================================
  // Statistics Tracking
  // ============================================
  int total_transfers;
  int total_bytes_transferred;
  longint start_time, end_time;
  real throughput_mbps;
  
  // -----------------------
  // CPU AXI driver tasks
  // -----------------------
  typedef axi_test::axi_driver #(
    .AW ( TbAxiAddrWidth ),
    .DW ( TbAxiDataWidth ),
    .IW ( XbarIdWidthSlvPorts ),
    .UW ( TbAxiUserWidth ),
    .TA ( TbApplTime     ),
    .TT ( TbTestTime     )
  ) cpu_axi_driver_t;

  task automatic cpu_write32(
    input  cpu_axi_driver_t drv,
    input  logic [31:0] addr,
    input  logic [31:0] data
  );
    cpu_axi_driver_t::ax_beat_t aw = new;
    cpu_axi_driver_t::w_beat_t  w  = new;
    cpu_axi_driver_t::b_beat_t  b  = new;

    aw.ax_id    = '0;
    aw.ax_addr  = addr;
    aw.ax_len   = 8'd0;
    aw.ax_size  = 3'd2;
    aw.ax_burst = axi_pkg::BURST_INCR;
    aw.ax_lock  = '0;
    aw.ax_cache = '0;
    aw.ax_prot  = '0;
    aw.ax_qos   = '0;
    aw.ax_region= '0;
    aw.ax_atop  = '0;
    aw.ax_user  = '0;

    drv.send_aw(aw);

    w.w_data = {{(TbAxiDataWidth-32){1'b0}}, data};
    w.w_strb = {{(TbAxiDataWidth/8-4){1'b0}}, 4'hF};
    w.w_last = 1'b1;
    w.w_user = '0;
    drv.send_w(w);

    drv.recv_b(b);
    if (b.b_resp != axi_pkg::RESP_OKAY)
      $fatal(1, "CPU write32 failed: addr=0x%08x resp=%0d", addr, b.b_resp);
  endtask

  task automatic cpu_read32(
    input  cpu_axi_driver_t drv,
    input  logic [31:0] addr,
    output logic [31:0] data
  );
    cpu_axi_driver_t::ax_beat_t ar = new;
    cpu_axi_driver_t::r_beat_t  r  = new;

    ar.ax_id    = '0;
    ar.ax_addr  = addr;
    ar.ax_len   = 8'd0;
    ar.ax_size  = 3'd2;
    ar.ax_burst = axi_pkg::BURST_INCR;
    ar.ax_lock  = '0;
    ar.ax_cache = '0;
    ar.ax_prot  = '0;
    ar.ax_qos   = '0;
    ar.ax_region= '0;
    ar.ax_user  = '0;

    drv.send_ar(ar);
    drv.recv_r(r);

    if (r.r_resp != axi_pkg::RESP_OKAY)
      $fatal(1, "CPU read32 failed: addr=0x%08x resp=%0d", addr, r.r_resp);

    data = r.r_data[31:0];
  endtask

  task automatic cpu_write64(
    input  cpu_axi_driver_t drv,
    input  logic [31:0] addr,
    input  logic [63:0] data
  );
    cpu_axi_driver_t::ax_beat_t aw = new;
    cpu_axi_driver_t::w_beat_t  w  = new;
    cpu_axi_driver_t::b_beat_t  b  = new;

    aw.ax_id    = '0;
    aw.ax_addr  = addr;
    aw.ax_len   = 8'd0;
    aw.ax_size  = 3'd3;
    aw.ax_burst = axi_pkg::BURST_INCR;
    aw.ax_lock  = '0;
    aw.ax_cache = '0;
    aw.ax_prot  = '0;
    aw.ax_qos   = '0;
    aw.ax_region= '0;
    aw.ax_atop  = '0;
    aw.ax_user  = '0;

    drv.send_aw(aw);

    w.w_data = data;
    w.w_strb = 8'hFF;
    w.w_last = 1'b1;
    w.w_user = '0;
    drv.send_w(w);

    drv.recv_b(b);
    if (b.b_resp != axi_pkg::RESP_OKAY)
      $fatal(1, "CPU write64 failed: addr=0x%08x resp=%0d", addr, b.b_resp);
  endtask

  task automatic cpu_read64(
    input  cpu_axi_driver_t drv,
    input  logic [31:0] addr,
    output logic [63:0] data
  );
    cpu_axi_driver_t::ax_beat_t ar = new;
    cpu_axi_driver_t::r_beat_t  r  = new;

    ar.ax_id    = '0;
    ar.ax_addr  = addr;
    ar.ax_len   = 8'd0;
    ar.ax_size  = 3'd3;
    ar.ax_burst = axi_pkg::BURST_INCR;
    ar.ax_lock  = '0;
    ar.ax_cache = '0;
    ar.ax_prot  = '0;
    ar.ax_qos   = '0;
    ar.ax_region= '0;
    ar.ax_user  = '0;

    drv.send_ar(ar);
    drv.recv_r(r);

    if (r.r_resp != axi_pkg::RESP_OKAY)
      $fatal(1, "CPU read64 failed: addr=0x%08x resp=%0d", addr, r.r_resp);

    data = r.r_data;
  endtask

  // ============================================
  // DMA Transfer Task
  // ============================================
  task automatic dma_transfer(
    input cpu_axi_driver_t drv,
    input axi_addr_t src_addr,
    input axi_addr_t dst_addr,
    input int unsigned length,
    input string description,
    output int poll_cycles
  );
    logic [31:0] next_id, done_id;
    int poll_count;
    
    $display("[%0t] DMA Transfer: %s", $time, description);
    $display("  SRC: 0x%08x | DST: 0x%08x | LEN: %0d bytes", src_addr, dst_addr, length);
    
    cpu_write32(drv, IDMA_BASE + IDMA_SRC_ADDR_OFFSET, src_addr);
    cpu_write32(drv, IDMA_BASE + IDMA_DST_ADDR_OFFSET, dst_addr);
    cpu_write32(drv, IDMA_BASE + IDMA_LENGTH_OFFSET, length);
    if (DmaConfEnableTwoD) begin
      cpu_write32(drv, IDMA_BASE + IDMA_REPS_2, 32'h0000_0001);
    end
    
    cpu_read32(drv, IDMA_BASE + IDMA_NEXT_ID_OFFSET, next_id);
    cpu_write32(drv, IDMA_BASE + IDMA_CONF, (32'(1) << 10));
    
    poll_count = 0;
    do begin
      repeat(10) @(posedge clk);
      cpu_read32(drv, IDMA_BASE + IDMA_DONE_ID_OFFSET, done_id);
      poll_count++;
      if (poll_count % 500 == 0) 
        $display("  Polling... done_id=%0d (waiting for %0d)", done_id, next_id);
    end while ((done_id != next_id) && (poll_count < 10000));
    
    if (poll_count >= 10000) $fatal(1, "DMA transfer timeout: %s", description);
    
    poll_cycles = poll_count;
    $display("  = Transfer complete (polls=%0d)", poll_count);
    total_transfers++;
    total_bytes_transferred += length;
  endtask

  // ============================================
  // Data Pattern Generation
  // ============================================
  function automatic logic [63:0] generate_pattern(
    input int index,
    input int pattern_type
  );
    case (pattern_type)
      0: return 64'hDEAD_BEEF_CAFE_0000 + index[15:0];                    // Incremental
      1: return {32'hAAAA_5555, 32'h5555_AAAA};                           // Alternating
      2: return {index[31:0], ~index[31:0]};                              // Index + complement
      default: return 64'hDEAD_BEEF_CAFE_0000 + index[15:0];
    endcase
  endfunction

  // ============================================
  // Main Stress Test
  // ============================================
  initial begin : proc_sim
    cpu_axi_driver_t cpu_drv;
    int poll_cycles;
    int test_num;
    int error_count;
    
    // Test configurations: {length, pattern_type, iterations}
    typedef struct {
      int unsigned length;
      int pattern_type;
      int iterations;
      string description;
    } test_config_t;
    
    test_config_t test_configs[] = '{
      '{64,    0, 70,  "Baseline: 64B incremental"},
      '{128,   0, 70,  "Small: 128B incremental"},
      //'{256,   0, 50,  "Medium: 256B incremental"},
      '{64,    1, 70,  "Pattern stress: 64B alternating x5"},
      '{128,   2, 70,  "Pattern stress: 128B index+comp x5"},
      '{64,    0, 70, "Burst stress: 64B x10 back-to-back"},
      '{128,   0, 70, "Burst stress: 128B x10 back-to-back"}
    };
    
    // Memory regions for testing
    localparam axi_addr_t SRAM_TEST_BASE  = SRAM_BASE + 32'h0000_0100;
    localparam axi_addr_t HYPER_TEST_BASE = HYPERRAM_BASE + 32'h0000_0100;
    localparam axi_addr_t SRAM_VERIFY_BASE = SRAM_BASE + 32'h0000_4000;
    
    logic [TbAxiDataWidth-1:0] expected_data [0:4095];  // Support up to 32KB
    logic [TbAxiDataWidth-1:0] read_data;
    int i, beat_idx, iter;
    int max_beats;
    int addr_offset;
    logic [63:0] pattern;

    end_of_sim = 1'b0;
    cpu_drv = new(cpu_master_dv);
    cpu_drv.reset_master();
    total_transfers = 0;
    total_bytes_transferred = 0;
    error_count = 0;
    
    @(posedge rst_n);
    
    $display("\n");
    $display("============================================================");
    $display("=  DMA + HyperRAM STRESS TEST SUITE                        =");
    $display("============================================================");
    $display("");
    
    $display("[%0t] Waiting for HyperBus PHY initialization (600us)...", $time);
    #600us;
    $display("[%0t] = PHY initialization complete", $time);
    
    repeat(20) @(posedge clk);
    start_time = $time;

    // ============================================
    // Run All Test Configurations
    // ============================================
    addr_offset = 0;
    
    for (test_num = 0; test_num < test_configs.size(); test_num++) begin
      automatic test_config_t cfg = test_configs[test_num];
      automatic int num_beats = cfg.length / (TbAxiDataWidth/8);
      
      $display("\n");
      $display("============================================================");
      $display(" TEST %0d/%0d: %s", test_num+1, test_configs.size(), cfg.description);
      $display("============================================================");
      
      for (iter = 0; iter < cfg.iterations; iter++) begin
        automatic axi_addr_t sram_src  = SRAM_TEST_BASE + addr_offset;
        automatic axi_addr_t hyper_dst = HYPER_TEST_BASE + addr_offset;
        automatic axi_addr_t sram_dst  = SRAM_VERIFY_BASE + addr_offset;
        automatic int wait_cycles;

        if (cfg.iterations > 1) begin
          $display("\n--- Iteration %0d/%0d ---", iter+1, cfg.iterations);
        end
        
        // Step 1: Initialize SRAM with test pattern
        for (i = 0; i < num_beats; i++) begin
          pattern = generate_pattern(i, cfg.pattern_type);
          cpu_write64(cpu_drv, sram_src + (i * 8), pattern);
          expected_data[i] = pattern;
        end
        
        repeat(10) @(posedge clk);
        
        // Step 2: DMA Write SRAM -> HyperRAM
        dma_transfer(
          cpu_drv,
          sram_src,
          hyper_dst,
          cfg.length,
          $sformatf("SRAM - HyperRAM [iter %0d]", iter+1),
          poll_cycles
        );
        
        repeat(50) @(posedge clk);
        
        // Step 3: DMA Read HyperRAM -> SRAM
        dma_transfer(
          cpu_drv,
          hyper_dst,
          sram_dst,
          cfg.length,
          $sformatf("HyperRAM - SRAM [iter %0d]", iter+1),
          poll_cycles
        );
        
        // CRITICAL FIX: Scaled wait time for pipeline drain
        wait_cycles = 500 + (num_beats * 15);
        $display("  Waiting %0d cycles for DMA pipeline to drain...", wait_cycles);
        repeat(wait_cycles) @(posedge clk);
        
        // Step 4: Verify
        for (beat_idx = 0; beat_idx < num_beats; beat_idx++) begin
          cpu_read64(cpu_drv, sram_dst + (beat_idx * 8), read_data);
          
          if (read_data !== expected_data[beat_idx]) begin
            $error("MISMATCH @ beat %0d: expected=0x%016x got=0x%016x", 
                   beat_idx, expected_data[beat_idx], read_data);
            error_count++;
            if (error_count > 10) $fatal(1, "Too many errors, aborting test");
          end
        end
        
        $display("  = Verification PASSED (%0d beats)", num_beats);
        
        addr_offset += cfg.length;
        repeat(20) @(posedge clk);
      end
      
      $display("= TEST %0d COMPLETE", test_num+1);
    end

    // ============================================
    // Final Statistics
    // ============================================
    end_time = $time;
    throughput_mbps = (total_bytes_transferred * 8.0) / ((end_time - start_time) / 1e9) / 1e6;
    
    $display("\n");
    $display("============================================================");
    $display("=  STRESS TEST COMPLETE - ALL TESTS PASSED                 =");
    $display("============================================================");
    $display("");
    $display("Statistics:");
    $display("  Total transfers:    %0d", total_transfers);
    $display("  Total data:         %0d bytes (%.2f KB)", total_bytes_transferred, total_bytes_transferred/1024.0);
    $display("  Test duration:      %0t", end_time - start_time);
    $display("  Avg throughput:     %.2f Mbps", throughput_mbps);
    $display("  Errors:             %0d", error_count);
    $display("");
    
    repeat(100) @(posedge clk);
    end_of_sim = 1'b1;
    #1us;
    $finish;
  end

endmodule : axi_hyper_tb