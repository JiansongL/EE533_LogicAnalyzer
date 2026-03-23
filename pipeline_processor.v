`timescale 1ns/1ps

module pipeline_processor
   #(
      parameter DATA_WIDTH = 64,
      parameter CTRL_WIDTH = DATA_WIDTH/8,
      parameter UDP_REG_SRC_WIDTH = 2
   )
   (
      // --- data path interface
      input  [DATA_WIDTH-1:0]             in_data,
      input  [CTRL_WIDTH-1:0]             in_ctrl,
      input                                in_wr,
      output                               in_rdy,

      output [DATA_WIDTH-1:0]             out_data,
      output [CTRL_WIDTH-1:0]             out_ctrl,
      output                               out_wr,
      input                                out_rdy,

      // --- Register interface
      input                               reg_req_in,
      input                               reg_ack_in,
      input                               reg_rd_wr_L_in,
      input  [`UDP_REG_ADDR_WIDTH-1:0]   reg_addr_in,
      input  [`CPCI_NF2_DATA_WIDTH-1:0]  reg_data_in,
      input  [UDP_REG_SRC_WIDTH-1:0]     reg_src_in,

      output                              reg_req_out,
      output                              reg_ack_out,
      output                              reg_rd_wr_L_out,
      output  [`UDP_REG_ADDR_WIDTH-1:0]  reg_addr_out,
      output  [`CPCI_NF2_DATA_WIDTH-1:0] reg_data_out,
      output  [UDP_REG_SRC_WIDTH-1:0]    reg_src_out,

      // misc
      input                               reset,
      input                               clk
   );

   //------------------------- Signals-------------------------------
   // SW registers
   wire [31:0] reg_dmem_data_lo;
   wire [31:0] reg_dmem_data_hi;
   wire [31:0] reg_dmem_addr;
   wire [31:0] reg_imem_addr;
   wire [31:0] reg_pipeline_c;
   wire [31:0] reg_proc_reset;

   // HW registers
   reg [31:0] reg_imem_out;
   reg [31:0] reg_inst_addr_lo;
   reg [31:0] reg_inst_addr_hi;
   reg [31:0] reg_wb_res_lo;
   reg [31:0] reg_wb_res_hi;
   reg [31:0] reg_pipeline_status;

   // Internal control/debug
   reg [31:0] inst_write_data;
   reg        inst_wr_en;
   reg [31:0] imem_addr_prev;
   reg        inst_write_pulse;

   // Logic-analyzer style trace capture (64 samples x 128 bits)
   localparam TRACE_PTR_W = 6;
   localparam TRACE_DEPTH = (1 << TRACE_PTR_W);
   reg [127:0] trace_mem [0:TRACE_DEPTH-1];
   reg [127:0] trace_sample;
   reg [TRACE_PTR_W-1:0] trace_wr_ptr;
   reg        trace_full;
   reg        trace_wrapped;

   wire       trace_enable;
   wire       trace_clear;
   wire       trace_freeze;
   wire       trace_view_en;
   wire [TRACE_PTR_W-1:0] trace_rd_ptr;
   wire [127:0] trace_live_bus;

   wire [63:0] inst_addr;
   wire [63:0] wb_res;
   wire        proc_rst;

   assign proc_rst = reset | reg_proc_reset[0];
   assign trace_enable = reg_pipeline_c[1];
   assign trace_clear  = reg_pipeline_c[2];
   assign trace_freeze = reg_pipeline_c[3];
   assign trace_view_en = reg_pipeline_c[4];
   assign trace_rd_ptr = reg_dmem_addr[TRACE_PTR_W-1:0];
   assign trace_live_bus = {inst_addr, wb_res};

   //------------------------- Datapath -------------------------------
   PipelinedDatapath processor_inst (
      .clk      (clk),
      .InstData (inst_write_data),
      .rst      (proc_rst),
      .wea      (inst_wr_en),
      .InstADDR (inst_addr),
      .WB_Res   (wb_res)
   );

   generic_regs
   #(
      .UDP_REG_SRC_WIDTH   (UDP_REG_SRC_WIDTH),
      .TAG                 (`PIPELINE_PROCESSOR_BLOCK_ADDR),
      .REG_ADDR_WIDTH      (`PIPELINE_PROCESSOR_REG_ADDR_WIDTH),
      .NUM_COUNTERS        (0),
      .NUM_SOFTWARE_REGS   (6),
      .NUM_HARDWARE_REGS   (6)
   ) module_regs (
      .reg_req_in       (reg_req_in),
      .reg_ack_in       (reg_ack_in),
      .reg_rd_wr_L_in   (reg_rd_wr_L_in),
      .reg_addr_in      (reg_addr_in),
      .reg_data_in      (reg_data_in),
      .reg_src_in       (reg_src_in),

      .reg_req_out      (reg_req_out),
      .reg_ack_out      (reg_ack_out),
      .reg_rd_wr_L_out  (reg_rd_wr_L_out),
      .reg_addr_out     (reg_addr_out),
      .reg_data_out     (reg_data_out),
      .reg_src_out      (reg_src_out),

      .counter_updates  (),
      .counter_decrement(),

      // Keep software register order aligned with XML (high address to low).
      .software_regs    ({reg_proc_reset, reg_pipeline_c, reg_imem_addr,
                          reg_dmem_addr, reg_dmem_data_hi, reg_dmem_data_lo}),

      // Keep hardware register order aligned with XML (high address to low).
      .hardware_regs    ({reg_pipeline_status, reg_wb_res_hi, reg_wb_res_lo,
                          reg_inst_addr_hi, reg_inst_addr_lo, reg_imem_out}),

      .clk              (clk),
      .reset            (reset)
   );

   // Use IMEM address changes as a one-cycle instruction write pulse.
   always @(posedge clk) begin
      if (reset) begin
         inst_write_data      <= 32'h0;
         inst_wr_en           <= 1'b0;
         imem_addr_prev       <= 32'h0;
         inst_write_pulse     <= 1'b0;
         reg_imem_out         <= 32'h0;
         reg_inst_addr_lo     <= 32'h0;
         reg_inst_addr_hi     <= 32'h0;
         reg_wb_res_lo        <= 32'h0;
         reg_wb_res_hi        <= 32'h0;
         reg_pipeline_status  <= 32'h0;
         trace_sample         <= 128'h0;
         trace_wr_ptr         <= {TRACE_PTR_W{1'b0}};
         trace_full           <= 1'b0;
         trace_wrapped        <= 1'b0;
      end else begin
         inst_wr_en       <= 1'b0;
         inst_write_pulse <= 1'b0;

         if (trace_clear) begin
            trace_wr_ptr  <= {TRACE_PTR_W{1'b0}};
            trace_full    <= 1'b0;
            trace_wrapped <= 1'b0;
         end else if (trace_enable && !trace_freeze) begin
            trace_mem[trace_wr_ptr] <= trace_live_bus;
            if (trace_wr_ptr == TRACE_DEPTH-1) begin
               trace_wr_ptr  <= {TRACE_PTR_W{1'b0}};
               trace_full    <= 1'b1;
               trace_wrapped <= 1'b1;
            end else begin
               trace_wr_ptr <= trace_wr_ptr + 1'b1;
            end
         end

         trace_sample <= trace_mem[trace_rd_ptr];

         if (reg_pipeline_c[0] && (reg_imem_addr != imem_addr_prev)) begin
            inst_write_data  <= reg_dmem_data_lo;
            inst_wr_en       <= 1'b1;
            inst_write_pulse <= 1'b1;
            reg_imem_out     <= reg_dmem_data_lo;
            imem_addr_prev   <= reg_imem_addr;
         end

         if (trace_view_en) begin
            // Read a captured sample selected by reg_dmem_addr[5:0]
            reg_inst_addr_lo <= trace_sample[31:0];
            reg_inst_addr_hi <= trace_sample[63:32];
            reg_wb_res_lo    <= trace_sample[95:64];
            reg_wb_res_hi    <= trace_sample[127:96];
            reg_imem_out     <= {24'h0, trace_full, trace_wrapped, trace_rd_ptr};
         end else begin
            // Default live view
            reg_inst_addr_lo <= inst_addr[31:0];
            reg_inst_addr_hi <= inst_addr[63:32];
            reg_wb_res_lo    <= wb_res[31:0];
            reg_wb_res_hi    <= wb_res[63:32];
         end

         reg_pipeline_status <= {16'h0, trace_full, trace_wrapped, trace_freeze,
                                 trace_enable, trace_view_en, trace_wr_ptr,
                                 proc_rst, reg_pipeline_c[0], inst_write_pulse,
                                 trace_clear, 1'b0};
      end
   end

   // Pass packets through unchanged while the register block controls the pipeline core.
   assign out_data = in_data;
   assign out_ctrl = in_ctrl;
   assign out_wr   = in_wr;
   assign in_rdy   = out_rdy;

endmodule
