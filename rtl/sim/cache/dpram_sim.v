// dpram_sim.v  (SIM ONLY)
//
// Behavioral stand-in for the VHDL `dpram` (rtl/bram.vhd -> dpram_dif ->
// altsyncram, BIDIR_DUAL_PORT). The ModelSim ASE *VHDL* altera_mf altsyncram
// model crashes on this width_byteena=1 config, so we model the megafunction's
// documented behavior directly. The behaviors that matter for the
// fill-vs-snoop coherency race are reproduced exactly:
//
//   * read latency = 1 cycle (M10K registers the address; outdata UNREGISTERED)
//   * same-port read-during-write  = NEW_DATA   (q shows the just-written data)
//   * mixed-port read-during-write = OLD_DATA   (a read on one port does NOT
//     see a same-cycle write on the other port) -- this is the altsyncram
//     default for BIDIR_DUAL_PORT and is the crux of the snoop-vs-fill race.
//   * simultaneous write to the same address from both ports: port B wins here
//     (silicon is "undefined"; B-wins is the benign case for the snoop path).
//
// enable_a/enable_b/cs_a/cs_b are left unconnected by cpu_cache_new (VHDL
// defaulted them to '1'), so we treat the RAM as always enabled/selected.

module dpram #(
  parameter addr_width    = 8,
  parameter data_width    = 8,
  parameter mem_init_file = " "
)(
  input                     clock,
  input  [addr_width-1:0]   address_a,
  input  [data_width-1:0]   data_a,
  input                     enable_a,
  input                     wren_a,
  output reg [data_width-1:0] q_a,
  input                     cs_a,
  input  [addr_width-1:0]   address_b,
  input  [data_width-1:0]   data_b,
  input                     enable_b,
  input                     wren_b,
  output reg [data_width-1:0] q_b,
  input                     cs_b
);

  reg [data_width-1:0] mem [0:(2**addr_width)-1];

  always @(posedge clock) begin
    // Port A read (registered, 1-cycle). NEW_DATA on a same-port write.
    if (wren_a) q_a <= data_a;
    else        q_a <= mem[address_a];

    // Port B read (registered, 1-cycle). NEW_DATA on its own write, but for a
    // *mixed-port* collision it must see OLD data: because the mem update below
    // is a non-blocking assign, mem[address_b] here is the pre-write value, so
    // a same-edge port-A write is NOT reflected -> OLD_DATA. Correct.
    if (wren_b) q_b <= data_b;
    else        q_b <= mem[address_b];

    // Writes (NBA). If both ports target the same address, port B wins.
    if (wren_a) mem[address_a] <= data_a;
    if (wren_b) mem[address_b] <= data_b;
  end

endmodule
