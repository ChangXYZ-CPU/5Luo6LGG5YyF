// Same-clock row-query response: caller holds x/y/ink/source stable for the
// locator's one-clock query latency. One 8px-cell inward margin, including y.
// No source/epoch match, invalid paper or missing row => NO ink, never fallback.
module shape_paper_roi_gate(
 input wire I_paper_valid,I_paper_busy,I_ink,
 input wire [18:0] I_span,
 input wire [8:0] I_ink_x,
 input wire [7:0] I_ink_y,I_paper_ymin,I_paper_ymax,
 input wire [31:0] I_ink_source,I_ink_epoch,I_paper_source,I_paper_epoch,
 output wire O_ink
);
 wire [8:0] x={1'b0,I_ink_x[8:1]};
 wire [7:0] y={1'b0,I_ink_y[7:1]};
 assign O_ink=I_ink && I_paper_valid && !I_paper_busy && I_span[18] &&
  I_ink_x<320 && I_ink_y<180 && I_ink_source==I_paper_source &&
  I_ink_epoch==I_paper_epoch && y>I_paper_ymin && y<I_paper_ymax &&
  x>I_span[17:9] && x<I_span[8:0];
endmodule
