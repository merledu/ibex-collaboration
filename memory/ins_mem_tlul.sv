module instr_mem_tlul
(
  input clock,
  input rst_ni,

  // tl-ul insterface
  input  tlul_pkg::tl_h2d_t xbar_to_iccm,
  output tlul_pkg::tl_d2h_t iccm_to_xbar
);

  logic        we;
  logic        req;
  logic [11:0] addr;
  logic [31:0] wdata;
  logic [31:0] wmask;
  logic [31:0] rdata;
  logic        rvalid; 
  logic [3:0]  data_we;


  always_ff @(posedge clock) begin
    if (!rst_ni) begin
      instr_valid <= 1'b0;
    end else if (we) begin
      instr_valid <= 1'b0;
    end else begin 
      instr_valid <= req;
    end
  end

  assign data_we[1:0] = (wmask_i[23:16] != 8'd0) ? 2'b11: 2'b00;
  assign data_we[3:2] = (wmask_i[31:24] != 8'd0) ? 2'b11: 2'b00; 

  DFFRAM inst_memory (
    .CLK    (clock),  // system clock
    .EN     (req),    // chip enable
    .WE     (we),     // write mask
    .DI     (wdata),  // data input
    .DO     (rdata),  // data output
    .A      (addr)    // address
  );

  tlul_sram_adapter #(
    .SramAw       (12),
    .SramDw       (32), 
    .Outstanding  (2),  
    .ByteAccess   (1),
    .ErrOnWrite   (0),  // 1: Writes not allowed, automatically error
    .ErrOnRead    (0)   // 1: Reads not allowed, automatically error  

  ) inst_mem (
    .clk_i     (clock),
    .rst_ni    (system_rst_ni),
    .tl_i      (xbar_to_iccm),
    .tl_o      (iccm_to_xbar), 
    .req_o     (req                     ),
    .gnt_i     (1'b1),
    .we_o      (),
    .addr_o    (tlul_addr),
    .wdata_o   (),
    .wmask_o   (),
    .rdata_i   ((rst_ni) ? tlul_data: '0),
    .rvalid_i  (instr_valid),
    .rerror_i  (2'b0)
  );

endmodule