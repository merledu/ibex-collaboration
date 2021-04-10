module data_mem
(
  input clock,
  input rst_ni,

  // tl-ul insterface
  input  tlul_pkg::tl_h2d_t xbar_to_dccm,
  output tlul_pkg::tl_d2h_t dccm_to_xbar
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
      rvalid <= 1'b0;
    end else if (we_i) begin
      rvalid <= 1'b0;
    end else begin 
      rvalid <= req;
    end
  end

  assign data_we[1:0] = (wmask_i[23:16] != 8'd0) ? 2'b11: 2'b00;
  assign data_we[3:2] = (wmask_i[31:24] != 8'd0) ? 2'b11: 2'b00; 
  
DFFRAMD dccm (

    .CLK    (clock  ),
    .EN     (req    ),   // chip enable
    .WE     (data_we),   // write mask
    .Di     (wdata  ),   // data input
    .Do     (rdata  ),   // data output
    .A      (addr   )    // address
);

tlul_sram_adapter #(
  .SramAw       (12),
  .SramDw       (32), 
  .Outstanding  (4),  
  .ByteAccess   (1),
  .ErrOnWrite   (0),  // 1: Writes not allowed, automatically error
  .ErrOnRead    (0) 

) data_mem (
    .clk_i    (clock       ),
    .rst_ni   (rst_ni      ),
    .tl_i     (xbar_to_dccm),
    .tl_o     (dccm_to_xbar), 
    .req_o    (req         ),
    .gnt_i    (1'b1        ),
    .we_o     (we          ),
    .addr_o   (addr        ),
    .wdata_o  (wdata       ),
    .wmask_o  (wmask       ),
    .rdata_i  (rdata       ),
    .rvalid_i (rvalid      ),
    .rerror_i (2'b0        )

);

endmodule