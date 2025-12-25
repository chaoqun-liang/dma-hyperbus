
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

  typedef logic [XbarIdWidthSlvPorts-1:0] axi_id_slv_t; // initiators into xbar
  typedef logic [XbarIdWidthMstPorts-1:0] axi_id_mst_t; // targets behind xbar

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
  // Debug Monitors
  // ============================================
  initial begin
    forever @(posedge clk) begin
      if (rst_n) begin
        // AW channel
        if (i_dut_if.i_dut.i_axi_slave.axi_req_i.aw_valid && 
            i_dut_if.i_dut.i_axi_slave.axi_rsp_o.aw_ready) begin
          $display("[%0t] HYPERBUS_AXI: AW handshake addr=0x%08x id=%0d", 
                   $time, 
                   i_dut_if.i_dut.i_axi_slave.axi_req_i.aw.addr,
                   i_dut_if.i_dut.i_axi_slave.axi_req_i.aw.id);
        end
        
        // AR channel
        if (i_dut_if.i_dut.i_axi_slave.axi_req_i.ar_valid && 
            i_dut_if.i_dut.i_axi_slave.axi_rsp_o.ar_ready) begin
          $display("[%0t] HYPERBUS_AXI: AR handshake addr=0x%08x id=%0d len=%0d size=%0d", 
                   $time, 
                   i_dut_if.i_dut.i_axi_slave.axi_req_i.ar.addr,
                   i_dut_if.i_dut.i_axi_slave.axi_req_i.ar.id,
                   i_dut_if.i_dut.i_axi_slave.axi_req_i.ar.len,
                   i_dut_if.i_dut.i_axi_slave.axi_req_i.ar.size);
        end
        
        // W channel
        if (i_dut_if.i_dut.i_axi_slave.axi_req_i.w_valid && 
            i_dut_if.i_dut.i_axi_slave.axi_rsp_o.w_ready) begin
          $display("[%0t] HYPERBUS_AXI: W handshake data=0x%016x last=%0b", 
                   $time,
                   i_dut_if.i_dut.i_axi_slave.axi_req_i.w.data,
                   i_dut_if.i_dut.i_axi_slave.axi_req_i.w.last);
        end
        
        // B channel
        if (i_dut_if.i_dut.axi_b_valid && i_dut_if.i_dut.axi_b_ready) begin
          $display("[%0t] HYPERBUS: B response (after CDC) error=%0b", 
                   $time, i_dut_if.i_dut.axi_b_error);
        end
        
        if (i_dut_if.i_dut.i_axi_slave.axi_rsp_o.b_valid && 
            i_dut_if.i_dut.i_axi_slave.axi_req_i.b_ready) begin
          $display("[%0t] HYPERBUS_AXI: B output to master id=%0d resp=%0d", 
                   $time,
                   i_dut_if.i_dut.i_axi_slave.axi_rsp_o.b.id,
                   i_dut_if.i_dut.i_axi_slave.axi_rsp_o.b.resp);
        end
        
        // R channel
        if (i_dut_if.i_dut.i_axi_slave.axi_rsp_o.r_valid && 
            i_dut_if.i_dut.i_axi_slave.axi_req_i.r_ready) begin
          $display("[%0t] HYPERBUS_AXI: R output to master data=0x%016x id=%0d resp=%0d last=%0b", 
                   $time,
                   i_dut_if.i_dut.i_axi_slave.axi_rsp_o.r.data,
                   i_dut_if.i_dut.i_axi_slave.axi_rsp_o.r.id,
                   i_dut_if.i_dut.i_axi_slave.axi_rsp_o.r.resp,
                   i_dut_if.i_dut.i_axi_slave.axi_rsp_o.r.last);
        end
        
        // Transaction CDC
        if (i_dut_if.i_dut.axi_trans_valid && i_dut_if.i_dut.axi_trans_ready) begin
          $display("[%0t] HYPERBUS: Trans CDC (sys?phy) cs=%0b", 
                   $time, i_dut_if.i_dut.axi_tf_cdc.cs);
        end
        
        if (i_dut_if.i_dut.phy_trans_valid && i_dut_if.i_dut.phy_trans_ready) begin
          $display("[%0t] HYPERBUS_PHY: Trans received from CDC", $time);
        end
        
        // PHY B response
        if (i_dut_if.i_dut.phy_b_valid && i_dut_if.i_dut.phy_b_ready) begin
          $display("[%0t] HYPERBUS_PHY: B response generated error=%0b", 
                   $time, i_dut_if.i_dut.phy_b_error);
        end
        
        // TX FIFO
        if (i_dut_if.i_dut.axi_tx_valid && i_dut_if.i_dut.axi_tx_ready) begin
          $display("[%0t] HYPERBUS: TX FIFO write (sys?phy) data=0x%08x last=%0b", 
                   $time, i_dut_if.i_dut.axi_tx.data[15:0], i_dut_if.i_dut.axi_tx.last);
        end
        
        if (i_dut_if.i_dut.phy_tx_valid && i_dut_if.i_dut.phy_tx_ready) begin
          $display("[%0t] HYPERBUS_PHY: TX data consumed data=0x%08x last=%0b", 
                   $time, i_dut_if.i_dut.phy_tx.data[15:0], i_dut_if.i_dut.phy_tx.last);
        end
        
        // RX FIFO
        if (i_dut_if.i_dut.phy_rx_valid && i_dut_if.i_dut.phy_rx_ready) begin
          $display("[%0t] HYPERBUS_PHY: RX data produced data=0x%08x last=%0b error=%0b", 
                   $time, 
                   i_dut_if.i_dut.phy_rx.data[15:0], 
                   i_dut_if.i_dut.phy_rx.last,
                   i_dut_if.i_dut.phy_rx.error);
        end
        
        if (i_dut_if.i_dut.axi_rx_valid && i_dut_if.i_dut.axi_rx_ready) begin
          $display("[%0t] HYPERBUS_AXI: RX data consumed data=0x%08x last=%0b", 
                   $time, 
                   i_dut_if.i_dut.axi_rx.data[15:0], 
                   i_dut_if.i_dut.axi_rx.last);
        end
      end
    end
  end

  // Watchdog for B response
  logic [31:0] b_wait_counter;
  logic        waiting_for_b;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      b_wait_counter <= 0;
      waiting_for_b  <= 1'b0;
    end else begin
      if (i_dut_if.i_dut.i_axi_slave.axi_req_i.aw_valid && 
          i_dut_if.i_dut.i_axi_slave.axi_rsp_o.aw_ready) begin
        waiting_for_b <= 1'b1;
        b_wait_counter <= 0;
        $display("[%0t] DEBUG: Started waiting for B response", $time);
      end
      
      if (i_dut_if.i_dut.i_axi_slave.axi_rsp_o.b_valid && 
          i_dut_if.i_dut.i_axi_slave.axi_req_i.b_ready) begin
        waiting_for_b <= 1'b0;
        $display("[%0t] DEBUG: B response received after %0d cycles", 
                 $time, b_wait_counter);
      end
      
      if (waiting_for_b) begin
        b_wait_counter <= b_wait_counter + 1;
        
        if (b_wait_counter == 100) 
          $warning("[%0t] Still waiting for B response after 100 cycles", $time);
        if (b_wait_counter == 500) 
          $warning("[%0t] Still waiting for B response after 500 cycles", $time);
        if (b_wait_counter == 1000) 
          $error("[%0t] B RESPONSE TIMEOUT after 1000 cycles!", $time);
      end
    end
  end

  // Monitor CDC FIFO states
  always @(posedge clk) begin
    if (rst_n && $time > 600us) begin
      if (i_dut_if.i_dut.i_cdc_2phase_b.src_valid_i && 
          !i_dut_if.i_dut.i_cdc_2phase_b.src_ready_o) begin
        $display("[%0t] WARNING: B CDC FIFO source side stuck (phy?sys)", $time);
      end
      
      if (i_dut_if.i_dut.i_cdc_fifo_tx.src_valid_i && 
          !i_dut_if.i_dut.i_cdc_fifo_tx.src_ready_o) begin
        $display("[%0t] WARNING: TX CDC FIFO stuck (sys?phy)", $time);
      end
      
      if (i_dut_if.i_dut.i_cdc_fifo_rx.src_valid_i && 
          !i_dut_if.i_dut.i_cdc_fifo_rx.src_ready_o) begin
        $display("[%0t] WARNING: RX CDC FIFO stuck (phy?sys)", $time);
      end
    end
  end

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

  // -----------------------
  // Main Test
  // -----------------------
      logic [63:0] test_read;

  initial begin : proc_sim
    cpu_axi_driver_t cpu_drv;
    logic [31:0] next_id_write, done_id, next_id_read;
    int poll_count;
    localparam int unsigned TransferLen = 32'h0000_0040;
    //localparam int unsigned TransferLen = 32'h0000_0008;  // Just 8 bytes (1 beat)
    localparam axi_addr_t SrcBase       = SRAM_BASE + 32'h0000_0100;
    localparam axi_addr_t HyperOffset   = 32'h100;
    localparam axi_addr_t DstBaseWrite  = HYPERRAM_BASE + HyperOffset;
    localparam axi_addr_t DstBaseRead   = SRAM_BASE + 32'h0000_4000;

    logic [TbAxiDataWidth-1:0] expected_data [0:(TransferLen/(TbAxiDataWidth/8))-1];
    logic [TbAxiDataWidth-1:0] read_data;
    int i, beat_idx;

    end_of_sim = 1'b0;
    cpu_drv = new(cpu_master_dv);
    cpu_drv.reset_master();
    @(posedge rst_n);
    
    $display("[%0t] Waiting for HyperBus PHY initialization (600us)...", $time);
    #600us;
    $display("[%0t] PHY initialization complete, starting test", $time);
    
    repeat(20) @(posedge clk);

    $display("===========================================");
    $display("= Bidirectional iDMA test: SRAM <-> HyperRAM =");
    $display("===========================================");
    
    // Step 1: Initialize source data
    $display("Initializing source data in SRAM @ 0x%08x (length %0d bytes)", 
             SrcBase, TransferLen);

    foreach (expected_data[k]) expected_data[k] = '0;

    for (i = 0; i < TransferLen; i += 8) begin
      logic [63:0] beat_data;
      int beat_index;

      beat_index = i / 8;
      beat_data = 64'hDEAD_BEEF_CAFE_0000 + beat_index[15:0];

      cpu_write64(cpu_drv, SrcBase + i, beat_data);

      expected_data[beat_index] = beat_data;
      $display("  Beat %0d @ 0x%08x: wrote 0x%016x", 
               beat_index, SrcBase + i, beat_data);
    end

    repeat(10) @(posedge clk);

    // Step 2: DMA SRAM -> HyperRAM (write)
    $display("\nStarting DMA: SRAM -> HyperRAM (write)");
    $display("  SRC: 0x%08x", SrcBase);
    $display("  DST: 0x%08x", DstBaseWrite);
    $display("  LEN: %0d bytes", TransferLen);
    
    cpu_write32(cpu_drv, IDMA_BASE + IDMA_SRC_ADDR_OFFSET, SrcBase);
    cpu_write32(cpu_drv, IDMA_BASE + IDMA_DST_ADDR_OFFSET, DstBaseWrite);
    cpu_write32(cpu_drv, IDMA_BASE + IDMA_LENGTH_OFFSET, TransferLen);
    if (DmaConfEnableTwoD) begin
      cpu_write32(cpu_drv, IDMA_BASE + IDMA_REPS_2, 32'h0000_0001);
    end
    
    cpu_read32(cpu_drv, IDMA_BASE + IDMA_NEXT_ID_OFFSET, next_id_write);
    $display("  Pre-transfer next_id=%0d", next_id_write);
    
    cpu_write32(cpu_drv, IDMA_BASE + IDMA_CONF, (32'(1) << 10));
    
    cpu_read32(cpu_drv, IDMA_BASE + IDMA_NEXT_ID_OFFSET, next_id_write);
    $display("  Transfer started. Expected completion ID=%0d", next_id_write);

    poll_count = 0;
    do begin
      repeat(10) @(posedge clk);
      cpu_read32(cpu_drv, IDMA_BASE + IDMA_DONE_ID_OFFSET, done_id);
      poll_count++;
      if (poll_count % 100 == 0) 
        $display("  Polling... done_id=%0d (waiting for %0d)", done_id, next_id_write);
    end while ((done_id != next_id_write) && (poll_count < 2000));
    
    if (poll_count >= 2000) $fatal(1, "DMA write transfer timeout");
    $display("Write transfer complete (done_id=%0d, polls=%0d)", done_id, poll_count);
    

    repeat(50) @(posedge clk);
    
    // Step 2.5: Test direct HyperRAM read
    $display("\n=== Testing direct HyperRAM read ===");
    
    $display("Attempting direct CPU read from HyperRAM @ 0x%08x", DstBaseWrite);
    cpu_read64(cpu_drv, DstBaseWrite, test_read);
    $display("Direct HyperRAM read result: 0x%016x (expected 0x%016x)", 
             test_read, expected_data[0]);
    
    if (test_read !== expected_data[0]) begin
      $error("Direct HyperRAM read failed! Data: expected=0x%016x got=0x%016x", 
             expected_data[0], test_read);
    end else begin
      $display("? Direct HyperRAM read SUCCESS - data was written correctly");
    end
    
    repeat(50) @(posedge clk);

    // Step 3: DMA HyperRAM -> SRAM (read back)
    $display("\nStarting DMA: HyperRAM -> SRAM (read back)");
    $display("  SRC: 0x%08x", DstBaseWrite);
    $display("  DST: 0x%08x", DstBaseRead);
    $display("  LEN: %0d bytes", TransferLen);
    
    cpu_write32(cpu_drv, IDMA_BASE + IDMA_SRC_ADDR_OFFSET, DstBaseWrite);
    cpu_write32(cpu_drv, IDMA_BASE + IDMA_DST_ADDR_OFFSET, DstBaseRead);
    cpu_write32(cpu_drv, IDMA_BASE + IDMA_LENGTH_OFFSET, TransferLen);
    if (DmaConfEnableTwoD) begin
      cpu_write32(cpu_drv, IDMA_BASE + IDMA_REPS_2, 32'h0000_0001);
    end
    
    cpu_read32(cpu_drv, IDMA_BASE + IDMA_NEXT_ID_OFFSET, next_id_read);
    $display("  Pre-transfer next_id=%0d", next_id_read);
    
    cpu_write32(cpu_drv, IDMA_BASE + IDMA_CONF, (32'(1) << 10));
    
    cpu_read32(cpu_drv, IDMA_BASE + IDMA_NEXT_ID_OFFSET, next_id_read);
    $display("  Transfer started. Expected completion ID=%0d", next_id_read);

    poll_count = 0;
    do begin
      repeat(10) @(posedge clk);
      cpu_read32(cpu_drv, IDMA_BASE + IDMA_DONE_ID_OFFSET, done_id);
      poll_count++;
      if (poll_count % 100 == 0) 
        $display("  Polling... done_id=%0d (waiting for %0d)", done_id, next_id_read);
    end while ((done_id != next_id_read) && (poll_count < 2000));
    
    if (poll_count >= 2000) $fatal(1, "DMA read transfer timeout");
    $display("Read transfer complete (done_id=%0d, polls=%0d)", done_id, poll_count);
    
    // CRITICAL FIX: Wait for data to actually propagate to SRAM!
  $display("Waiting for read data to propagate through DMA pipeline...");
  repeat(100) @(posedge clk);  // Add sufficient delay

    repeat(20) @(posedge clk);

    // Step 4: Verify data
    $display("\nVerifying read-back data @ 0x%08x", DstBaseRead);
    for (beat_idx = 0; beat_idx < TransferLen / (TbAxiDataWidth/8); beat_idx++) begin
      axi_addr_t beat_addr;

      beat_addr = DstBaseRead + beat_idx * (TbAxiDataWidth/8);
      
      cpu_read64(cpu_drv, beat_addr, read_data);

      $display("  Beat %0d @ 0x%08x: expected 0x%016x, got 0x%016x %s",
               beat_idx, beat_addr, expected_data[beat_idx], read_data,
               (read_data === expected_data[beat_idx]) ? "PASS" : "FAIL");

      if (read_data !== expected_data[beat_idx]) begin
        $fatal(1, "Data mismatch at beat %0d", beat_idx);
      end
    end

    $display("\n=== Verification PASSED: Bidirectional DMA transfer successful! ===");
    repeat(100) @(posedge clk);
    end_of_sim = 1'b1;
    #1us;
    $finish;
  end

endmodule : axi_hyper_tb