////////////////////////////////////////////////////////////////////////////////
//
// Description: The top -level FIFO module is a parameterized FIFO design with all sub-blocks 
//              instantiated using the recommended practice of doing named port connections.
// Reference  : Simulation and Synthesis Techniques for Asynchronous FIFO Design 
//              Clifford E. Cummings, Sunburst Design, Inc. 
//              cliffc@sunburst-design.com
//
////////////////////////////////////////////////////////////////////////////////
module axis_fifo #(
    parameter DATA_WIDTH = 8,
    parameter ADDR_DEPTH = 4
) (
    input  wire                    s_aclk,
    input  wire                    s_areset_n,
    input  wire [DATA_WIDTH-1:0]   s_tdata,
    input  wire [DATA_WIDTH/8-1:0] s_tkeep,
    input  wire                    s_tvalid,
    output wire                    s_tready,
    input  wire                    s_tlast,

    input  wire                    m_aclk,
    input  wire                    m_areset_n,
    output wire [DATA_WIDTH-1:0]   m_tdata,
    output wire [DATA_WIDTH/8-1:0] m_tkeep,
    output wire                    m_tvalid,
    input  wire                    m_tready,
    output wire                    m_tlast
);

wire [DATA_WIDTH+DATA_WIDTH/8+1-1:0] m_tdata_int;
wire s_full;
wire m_empty;

fifo #(
    .DSIZE  ( DATA_WIDTH ) ,
    .ASIZE  ( ADDR_DEPTH )
) fifo (
    .wfull  ( s_full ) ,
    .wdata  ( {s_tlast, s_tkeep, s_tdata} ) ,
    .winc   ( s_tvalid & s_tready ) , 
    .wclk   ( s_aclk ) , 
    .wrst_n ( s_areset_n ) ,
    .rempty ( m_empty ) ,
    .rdata  ( m_tdata_int ) ,
    .rinc   ( m_tvalid & m_tready ) , 
    .rclk   ( m_aclk ) , 
    .rrst_n ( m_areset_n )
);

assign s_tready = ~s_full; 
assign m_tvalid = ~m_empty;
assign m_tdata  = m_tdata_int[DATA_WIDTH-1:0];
assign m_tkeep  = m_tdata_int[DATA_WIDTH+DATA_WIDTH/8-1:DATA_WIDTH];
assign m_tlast  = m_tdata_int[DATA_WIDTH+DATA_WIDTH/8];

endmodule
////////////////////////////////////////////////////////////////////////////////
// FIFO top-level module
////////////////////////////////////////////////////////////////////////////////
module fifo #(
    parameter DSIZE = 8,
    parameter ASIZE = 4
) (
    output [DSIZE-1:0] rdata,
    output             wfull,
    output             rempty,
    input  [DSIZE-1:0] wdata,
    input              winc, wclk, wrst_n,
    input              rinc, rclk, rrst_n
);

wire   [ASIZE-1:0] waddr, raddr;
wire   [ASIZE:0]   wptr, rptr, wq2_rptr, rq2_wptr;

sync_r2w sync_r2w ( .wq2_rptr(wq2_rptr), .rptr(rptr),
    .wclk(wclk), .wrst_n(wrst_n) );

sync_w2r sync_w2r ( .rq2_wptr(rq2_wptr), .wptr(wptr),
    .rclk(rclk), .rrst_n(rrst_n) );

fifomem #(DSIZE, ASIZE) fifomem (
    .rdata(rdata), .wdata(wdata), .waddr(waddr), 
    .raddr(raddr), .wclken(winc), .wfull(wfull),
    .wclk(wclk) );

rptr_empty #(ASIZE) rptr_empty (
    .rempty(rempty), .raddr(raddr), .rptr(rptr), 
    .rq2_wptr(rq2_wptr), .rinc(rinc), .rclk(rclk), 
    .rrst_n(rrst_n) );

wptr_full #(ASIZE) wptr_full (
    .wfull(wfull), .waddr(waddr), .wptr(wptr), 
    .wq2_rptr(wq2_rptr), .winc(winc),  .wclk(wclk),
    .wrst_n(wrst_n) );

endmodule
////////////////////////////////////////////////////////////////////////////////
// FIFO memory buffer
////////////////////////////////////////////////////////////////////////////////
module fifomem #(
    parameter  DATASIZE = 8,
    parameter  ADDRSIZE = 4
) (
    output [DATASIZE-1:0] rdata,
    input  [DATASIZE-1:0] wdata,
    input  [ADDRSIZE-1:0] waddr, raddr,
    input                 wclken, wfull, wclk
);
`ifdef VENDORRAM
    // instantiation of a vendor's dual-port RAM
    vendor_ram mem (.dout(rdata), .din(wdata),
        .waddr(waddr), .raddr(raddr),
        .wclken(wclken), .wclken_n(wfull), .clk(wclk)
    );
`else
    // RTL Verilog memory model
    localparam DEPTH = 1<<ADDRSIZE;
    reg [DATASIZE-1:0] mem [0:DEPTH-1];

    assign rdata = mem[raddr];

    always @(posedge wclk) begin
        if (wclken && !wfull) begin
            mem[waddr] <= wdata;
        end
    end
`endif
endmodule
////////////////////////////////////////////////////////////////////////////////
// Read-domain to write-domain synchronizer
////////////////////////////////////////////////////////////////////////////////
module sync_r2w #( parameter ADDRSIZE = 4 ) (
    output reg [ADDRSIZE:0] wq2_rptr,
    input      [ADDRSIZE:0] rptr,
    input                   wclk, wrst_n
);

reg [ADDRSIZE:0] wq1_rptr;

always @(posedge wclk or negedge wrst_n) begin
    if (!wrst_n) begin
        {wq2_rptr,wq1_rptr} <= 0;
    end else begin
        {wq2_rptr,wq1_rptr} <= {wq1_rptr,rptr};
    end
end

endmodule
////////////////////////////////////////////////////////////////////////////////
// Write-domain to read-domain synchronizer
////////////////////////////////////////////////////////////////////////////////
module sync_w2r #( parameter ADDRSIZE = 4 ) (
    output reg [ADDRSIZE:0] rq2_wptr,
    input      [ADDRSIZE:0] wptr,
    input                   rclk, rrst_n
);

reg [ADDRSIZE:0] rq1_wptr;

always @(posedge rclk or negedge rrst_n) begin
    if (!rrst_n) begin
        {rq2_wptr,rq1_wptr} <= 0;
    end else begin
        {rq2_wptr,rq1_wptr} <= {rq1_wptr,wptr};
    end
end

endmodule
////////////////////////////////////////////////////////////////////////////////
// Read pointer & empty generation logic
////////////////////////////////////////////////////////////////////////////////
module rptr_empty #( parameter ADDRSIZE = 4 ) (
    output reg                rempty,
    output     [ADDRSIZE-1:0] raddr,
    output reg [ADDRSIZE  :0] rptr,
    input      [ADDRSIZE  :0] rq2_wptr,
    input                     rinc, rclk, rrst_n
);

reg  [ADDRSIZE:0] rbin;
wire [ADDRSIZE:0] rgraynext, rbinnext;
//-------------------
// GRAYSTYLE2 pointer
//-------------------
always @(posedge rclk or negedge rrst_n) begin
    if (!rrst_n) begin
        {rbin, rptr} <= 0;
    end else begin
        {rbin, rptr} <= {rbinnext, rgraynext};
    end
end
// Memory read-address pointer (okay to use binary to address memory)
assign raddr     = rbin[ADDRSIZE-1:0];
assign rbinnext  = rbin + (rinc & ~rempty);
assign rgraynext = (rbinnext>>1) ^ rbinnext;
//---------------------------------------------------------------
// FIFO empty when the next rptr == synchronized wptr or on reset
//---------------------------------------------------------------
assign rempty_val = (rgraynext == rq2_wptr);

always @(posedge rclk or negedge rrst_n) begin
    if (!rrst_n) begin
        rempty <= 1'b1;
    end else begin        
        rempty <= rempty_val;
    end
end

endmodule
////////////////////////////////////////////////////////////////////////////////
// Write pointer & full generation logic
////////////////////////////////////////////////////////////////////////////////
module wptr_full  #( parameter ADDRSIZE = 4 ) (
    output reg                wfull,
    output     [ADDRSIZE-1:0] waddr,
    output reg [ADDRSIZE  :0] wptr,
    input      [ADDRSIZE  :0] wq2_rptr,
    input                     winc, wclk, wrst_n
);

reg  [ADDRSIZE:0] wbin;
wire [ADDRSIZE:0] wgraynext, wbinnext;
// GRAYSTYLE2 pointer
always @(posedge wclk or negedge wrst_n) begin
    if (!wrst_n) begin
        {wbin, wptr} <= 0;
    end else begin
        {wbin, wptr} <= {wbinnext, wgraynext};
    end
end
// Memory write-address pointer (okay to use binary to address memory)
assign waddr = wbin[ADDRSIZE-1:0];
assign wbinnext  = wbin + (winc & ~wfull);
assign wgraynext = (wbinnext>>1) ^ wbinnext;
//------------------------------------------------------------------
// Simplified version of the three necessary full-tests:
// assign wfull_val=((wgnext[ADDRSIZE]    !=wq2_rptr[ADDRSIZE]  ) &&
//                   (wgnext[ADDRSIZE-1]  !=wq2_rptr[ADDRSIZE-1]) &&
//                   (wgnext[ADDRSIZE-2:0]==wq2_rptr[ADDRSIZE-2:0]));
//------------------------------------------------------------------
assign wfull_val = (wgraynext=={~wq2_rptr[ADDRSIZE:ADDRSIZE-1], wq2_rptr[ADDRSIZE-2:0]});

always @(posedge wclk or negedge wrst_n) begin
    if (!wrst_n) begin 
        wfull  <= 1'b0;
    end else begin
        wfull  <= wfull_val;
    end
end

endmodule
