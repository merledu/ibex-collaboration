 
module azadi_soc_top(
  input clk_i,
  input rst_ni,

  input  logic [19:0] gpio_i,
  output logic [19:0] gpio_o,

);

  wire [19:0] gpio_in;
  wire [19:0] gpio_out;

  assign gpio_in = gpio_i;
  assign gpio_o = gpio_out; 
        
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
      .DmHaltAddr       (), 
      .DmExceptionAddr  () 
  ) u_top (
      .clk_i  (clk_i),
      .rst_ni (rst_ni),

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

      // CPU Control Signals
      .fetch_enable_i (1'b1),
      .alert_minor_o  (),
      .alert_major_o  (),
      .core_sleep_o   ()
  );

  //peripheral xbar
  xbar_periph periph_switch (
    .clk_i     (clk_i),
    .rst_ni    (rst_ni),

    /* Host interfaces */
    .tl_if_i   (ifu_to_xbar), 
    .tl_if_o   (xbar_to_ifu), 
    .tl_lsu_i  (lsu_to_xbar),
    .tl_lsu_o  (xbar_to_lsu),

    /* Device interfaces */

    // GPIOs
    .tl_gpio_o  (xbarp_to_gpio),
    .tl_gpio_i  (gpio_to_xbarp),

    // LDO 1
    .tl_ldo1_o  (),
    .tl_ldo1_i  (),

    // LDO 2
    .tl_ldo2_o  (),
    .tl_ldo2_i  (),

    // DCDC
    .tl_dcdc_o  (),
    .tl_dcdc_o  (),

    // PLL 1
    .tl_pll1_o  (),
    .tl_pll1_i  (),

    // Temp. Sensor 1
    .tl_tmp_sens1_o  (),
    .tl_tmp_sens1_i  (),

    // Temp. Sensor 2
    .tl_tmp_sens2_o  (),
    .tl_tmp_sens2_i  (),
  
    // DAP
    .tl_dap_o         (),
    .tl_dap_i         (),

    // PLIC
    .tl_plic_o  (plic_req),
    .tl_plic_i  (plic_resp)
  );

  //GPIO module
   gpio gpio_32 (
    .clk_i         (clk_i),
    .rst_ni        (rst_ni),

    // Below Regster interface can be changed
    .tl_i          (xbarp_to_gpio),
    .tl_o          (gpio_to_xbarp),
    .cio_gpio_i    (gpio_in),
    .cio_gpio_o    (gpio_out),
    .cio_gpio_en_o (),
    .intr_gpio_o   (intr_gpio )  
  );

  instr_mem_tlul iccm (
    .clk_i    (clk_i),
    .rst_ni   (rst_ni),

    // tl-ul insterface
    .tl_d_i   (xbar_to_iccm),
    .tl_d_o   (iccm_to_xbar)
  );

  data_mem_tlul dccm(
    .clk_i    (clk_i),
    .rst_ni   (rst_ni),
  
    // tl-ul insterface
    .tl_d_i   (xbar_to_dccm),
    .tl_d_o   (dccm_to_xbar)
  );

  rv_plic intr_controller (
    .clk_i      (clk_i),
    .rst_ni     (rst_ni),

    // Bus Interface (device)
    .tl_i       (plic_req),
    .tl_o       (plic_resp),

    // Interrupt Sources
    .intr_src_i (intr_vector),

    // Interrupt notification to targets
    .irq_o      (intr_req),
    .irq_id_o   (),

    .msip_o     ()
  );

endmodule
