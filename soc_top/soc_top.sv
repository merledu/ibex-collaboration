 
module azadi_soc_top #(
  
  parameter logic [31:0] JTAG_ID = 32'h 0000_0001,
  parameter logic DirectDmiTap = 1'b1
)(
  input clock,
  input rst_ni,

  input  logic [19:0] gpio_i,
  output logic [19:0] gpio_o,

);

wire [19:0] gpio_in;
wire [19:0] gpio_out;

assign gpio_in = gpio_i;
assign gpio_o = gpio_out; 


// end here
        
  tlul_pkg::tl_h2d_t ifu_to_xbar;
  tlul_pkg::tl_d2h_t xbar_to_ifu;

  tlul_pkg::tl_h2d_t xbar_to_iccm;
  tlul_pkg::tl_d2h_t iccm_to_xbar;

  tlul_pkg::tl_h2d_t lsu_to_xbar;
  tlul_pkg::tl_d2h_t xbar_to_lsu;

  tlul_pkg::tl_h2d_t xbar_to_dccm;
  tlul_pkg::tl_d2h_t dccm_to_xbar;

  tlul_pkg::tl_h2d_t xbar_to_gpio;
  tlul_pkg::tl_d2h_t gpio_to_xbar;

  tlul_pkg::tl_h2d_t xbar_to_ldo1;
  tlul_pkg::tl_d2h_t ldo1_to_xbarm;

  tlul_pkg::tl_h2d_t xbar_to_ldo2;
  tlul_pkg::tl_d2h_t ldo2_to_xbar;

  tlul_pkg::tl_h2d_t xbar_to_dcdc;
  tlul_pkg::tl_d2h_t dcdc_to_xbar;

  tlul_pkg::tl_h2d_t xbar_to_pll1;
  tlul_pkg::tl_d2h_t pll1_to_xbar;

  tlul_pkg::tl_h2d_t xbar_to_tsen1;
  tlul_pkg::tl_d2h_t tsen1_to_xbar;

  tlul_pkg::tl_h2d_t xbar_to_tsen2;
  tlul_pkg::tl_d2h_t tsen2_to_xbar;

  tlul_pkg::tl_h2d_t xbar_to_dap;
  tlul_pkg::tl_d2h_t dap_to_xbar;

  tlul_pkg::tl_h2d_t plic_req;
  tlul_pkg::tl_d2h_t plic_resp;

  // interrupt vector
  logic [32:0] intr_vector;  // size depend on number of interrupts 
                             // increses on adding peripherals 

  // Interrupt source list 
  logic [31:0] intr_gpio;

  assign intr_vector = {  

      // add more pheripheral intrupts here
      intr_gpio,
      1'b0
  };

  logic [31:0] gpio_intr;
  logic       rx_dv_i;
  logic [7:0] rx_byte_i;


logic instr_valid;
logic [11:0] tlul_addr;
logic req_i;
logic [31:0] tlul_data;

logic iccm_cntrl_reset;
logic [11:0] iccm_cntrl_addr;
logic [31:0] iccm_cntrl_data;
logic iccm_cntrl_we;

brq_core_top #(
    .PMPEnable        (1'b0),
    .PMPGranularity   (0), 
    .PMPNumRegions    (0), 
    .MHPMCounterNum   (0), 
    .MHPMCounterWidth (40), 
    .RV32E            (1'b0), 
    .RV32M            (brq_pkg::RV32MFast), 
    .RV32B            (brq_pkg::RV32BNone), 
    .RegFile          (brq_pkg::RegFileFF), 
    .BranchTargetALU  (1'b0), 
    .WritebackStage   (1'b1), 
    .ICache           (1'b0), 
    .ICacheECC        (1'b0), 
    .BranchPredictor  (1'b0), 
    .DbgTriggerEn     (1'b1), 
    .DbgHwBreakNum    (2), 
    .Securebrq        (1'b0),
    .DmHaltAddr       (tl_main_pkg::ADDR_SPACE_DEBUG_ROM + 32'h 800), 
    .DmExceptionAddr  (tl_main_pkg::ADDR_SPACE_DEBUG_ROM + dm::ExceptionAddress) 
) u_top (
    .clock (clock),
    .reset (rst_ni),

  // instruction memory interface 
    .tl_i_i (xbar_to_ifu),
    .tl_i_o (ifu_to_xbar),

  // data memory interface 
    .tl_d_i (xbar_to_lsu),
    .tl_d_o (lsu_to_xbar),

    .test_en_i   (1'b0),     // enable all clock gates for testing

    .hart_id_i   (32'b0), 
    .boot_addr_i (32'h20000000),

        // Interrupt inputs
    .irq_software_i (1'b0),
    .irq_timer_i    (intr_timer),
    .irq_external_i (intr_req),
    .irq_fast_i     (1'b0),
    .irq_nm_i       (1'b0),       // non-maskeable interrupt

    // Debug Interface
    .debug_req_i    (dbg_req),
        // CPU Control Signals
    .fetch_enable_i (1'b1),
    .alert_minor_o  (),
    .alert_major_o  (),
    .core_sleep_o   ()
);

// main xbar module
  tl_xbar_main main_swith (
  .clk_main_i         (clock),
  .rst_main_ni        (rst_ni),

  // Host interfaces
  .tl_brqif_i         (ifu_to_xbar),
  .tl_brqif_o         (xbar_to_ifu),
  .tl_brqlsu_i        (lsu_to_xbar),
  .tl_brqlsu_o        (xbar_to_lsu),

  // Device interfaces
  .tl_iccm_o          (xbar_to_iccm),
  .tl_iccm_i          (iccm_to_xbar),
  .tl_dccm_o          (xbar_to_dccm),
  .tl_dccm_i          (dccm_to_xbar),
  .tl_plic_o          (plic_req),
  .tl_plic_i          (plic_resp),
  .tl_xbar_peri_o     (xbarm_to_xbarp),
  .tl_xbar_peri_i     (xbarp_to_xbarm),

  .scanmode_i         ()
);

// dummy data memory

data_mem dccm(
  .clock    (clock),
  .reset    (rst_ni),

// tl-ul insterface
  .tl_d_i   (xbar_to_dccm),
  .tl_d_o   (dccm_to_xbar)
);

//peripheral xbar

xbar_periph periph_switch (
  .clk_peri_i         (clock),
  .rst_peri_ni        (rst_ni),

  /* Host interfaces */
  .tl_if_i            (), 
  .tl_if_o            (), 
  .tl_lsu_i           (),
  .tl_lsu_o           (),

  /* Device interfaces */

  // GPIOs
  .tl_gpio_o          (xbarp_to_gpio),
  .tl_gpio_i          (gpio_to_xbarp),
  
  // LDO 1
  .tl_ldo1_o          ( ),
  .tl_ldo1_i          ( ),

  // LDO 2
  .tl_ldo2_o          ( ),
  .tl_ldo2_i          ( ),

  // DCDC
  .tl_dcdc_o          ( ),
  .tl_dcdc_o          ( ),

  // PLL 1
  .tl_pll1_o          ( ),
  .tl_pll1_i          ( ),

  // Temp. Sensor 1
  .tl_tmp_sens1_o     ( ),
  .tl_tmp_sens1_i     ( ),

  // Temp. Sensor 2
  .tl_tmp_sens2_o     ( ),
  .tl_tmp_sens2_i     ( ),
 
  // DAP
  .tl_dap_o           ( ),
  .tl_dap_i           ( ),

  .scanmode_i         ()
);

//GPIO module
 gpio gpio_32 (
  .clk_i          (clock),
  .rst_ni         (rst_ni),

  // Below Regster interface can be changed
  .tl_i           (xbarp_to_gpio),
  .tl_o           (gpio_to_xbarp),

  .cio_gpio_i     (gpio_in),
  .cio_gpio_o     (gpio_out),
  .cio_gpio_en_o  (),

  .intr_gpio_o    (intr_gpio )  
);

instr_mem_tlul iccm (
  .clock      (clock),
  .reset      (rst_ni),

  // tl-ul insterface
  .tl_d_i   (xbar_to_iccm),
  .tl_d_o   (iccm_to_xbar)
);

data_mem_tlul dccm(
  .clock    (clock),
  .reset    (rst_ni),
 
  // tl-ul insterface
  .tl_d_i   (xbar_to_dccm),
  .tl_d_o   (dccm_to_xbar)
);

rv_plic intr_controller (
  .clk_i(clock),
  .rst_ni(rst_ni),

  // Bus Interface (device)
  .tl_i (plic_req),
  .tl_o (plic_resp),

  // Interrupt Sources
  .intr_src_i (intr_vector),

  // Interrupt notification to targets
  .irq_o (intr_req),
  .irq_id_o(),

  .msip_o()
);

endmodule
