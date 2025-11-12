// input  stream: compressed bytes (AXI‑Stream‑like)
input  logic             in_valid_i;
output logic             in_ready_o;
input  logic [63:0]      in_data_i;       // 8 bytes/beat
input  logic             in_last_i;
input  logic [2:0]       in_last_bytes_i; // 1–8 valid bytes if last

// output stream: decompressed elements (packed)
output logic             out_valid_o;
input  logic             out_ready_i;
output logic [OUT_W-1:0] out_data_o;      // OUT_ELEMS × ELEM_W bits
output logic             out_last_o;
output logic [$clog2(OUT_ELEMS+1)-1:0] out_last_elems_o;
