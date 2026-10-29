// ============================================================================
//  layer_mixer.v -- multi-layer pixel compositor  (Task A, CORE MODULE)
//
//  This is the single insertion point of the whole project. Everything the
//  team adds later -- subtitles, transitions, OSD, audio spectrum -- flows
//  through here. The interface is frozen by 接口契约与分工表.md section 4.1.
//
//  Layer stack, bottom to top:
//     1. base_rgb                : the picture read out of SDRAM (old buffer)
//     2. scale_rgb               : the picture being blended in (new buffer)
//        -> layers 1+2 are combined by trans_alpha (the transition)
//     3. text_rgb / text_alpha   : subtitle band
//     4. spec_rgb / spec_alpha   : audio spectrum bars (provided by person C)
//     5. brightness / contrast   : applied LAST so it also affects the text
//
//  Blending maths
//  --------------
//  Alpha blend of one channel:
//      out = (a * fg + (256 - a) * bg) >> 8
//  With a=0 -> bg, a=255 -> (255*fg + bg)>>8, which is fg within 1 LSB.
//  >>8 is a shift, not a division: zero cost.
//
//  Where the multipliers go (LUT budget matters on EG4S20)
//  ------------------------------------------------------
//    transition : 3 multipliers (one per colour channel)
//    text       : NO multiplier -- text_alpha is always 0 or 255, so the
//                 blend collapses to a 2:1 multiplexer
//    spectrum   : treated the same way (binary alpha), so also a mux
//    brightness/contrast : 3 multipliers
//  Total: 6 multipliers per pixel. At 25 MHz this is a small pipeline and
//  leaves the LUTs free for the scaler that person B is writing.
//
//  Pipeline delay
//  --------------
//  The module is registered: outputs appear 1 clock after the inputs for that
//  pixel. The caller must therefore delay hs/vs along with it if it needs them
//  to stay aligned. For the example design this is not required because
//  video_rgb_to_axis only uses de/vs edges at frame rate.
// ============================================================================

`timescale 1ns / 1ps

module layer_mixer #(
    parameter H_ACTIVE = 640,
    parameter V_ACTIVE = 480,

    // ---- transition weights, out of 256 (keep the sum at 256) ----
    parameter W_TRANS_BASE = 128,       // old picture contribution
    parameter W_TRANS_MIX  = 128,       // new picture contribution
    parameter W_TEXT_BASE  = 128,
    parameter W_TEXT_MIX   = 128,
    parameter W_SPEC_BASE  = 192,       // keep the spectrum subtle
    parameter W_SPEC_MIX   = 64,

    // ---- display enhancement defaults (128 = neutral) ----
    parameter DEF_BRIGHT = 128,
    parameter DEF_CONTRAST = 128,

    // ---- spectrum overlay geometry ----
    parameter SPEC_W      = 256,        // width  of the spectrum area, pixels
    parameter SPEC_H      = 48,         // height of the spectrum area, pixels
    parameter SPEC_X      = 192,        // left edge
    parameter SPEC_Y      = 360,        // top edge

    parameter USE_TEXT    = 1,
    parameter USE_SPEC    = 1,
    parameter USE_OSD     = 1,
    parameter USE_ENHANCE = 1,
    // Fade-through-black transition. See stage 5 for why a crossfade between
    // two frames is not possible in this design and this is used instead.
    parameter USE_FADE    = 0,
    // Radial iris. See stage 6.
    parameter USE_IRIS    = 0
)(
    input  wire        clk,
    input  wire        rst,

    // ---------------- pixel position ----------------
    input  wire [9:0]  x,
    input  wire [9:0]  y,
    input  wire        de,              // wire to de_r from video_delay

    // ---------------- layer data ----------------
    input  wire [23:0] base_rgb,        // from video_delay (vout_data_raw)
    input  wire [23:0] scale_rgb,       // from the scaler, or wire to base_rgb
    input  wire [23:0] text_rgb,
    input  wire [7:0]  text_alpha,
    input  wire [23:0] spec_rgb,        // from audio_viz (person C)
    input  wire [7:0]  spec_alpha,
    input  wire [23:0] osd_rgb,         // from osd_bar (person B): parameter bar
    input  wire [7:0]  osd_alpha,

    // ---------------- control ----------------
    input  wire [7:0]  trans_alpha,     // 0 = old, 255 = new (stage 5 dimmer)

    // ---------------- radial iris (stage 6) ----------------
    //   iris_en       : 1 while the iris transition is playing
    //   iris_aperture : the aperture radius, IN THE SAME UNITS AS iris_d
    //                   0 = fully closed (whole frame black)
    //                   IRIS_DMAX = fully open (mask invisible)
    //
    // The aperture is passed in directly rather than as a 0..1 progress value.
    // That removes every conversion from this module, which is where the
    // calibration went wrong three times: a progress value has to be scaled
    // into metric units, and getting that scale wrong makes the mask either
    // invisible or permanently black with no clue in the code.
    input  wire        iris_en,
    input  wire [31:0] iris_aperture,

    input  wire        bypass,          // 1 = pass base_rgb straight through
    input  wire [7:0]  brightness,      // 0..255, 128 = neutral
    input  wire [7:0]  contrast,        // 0..255, 128 = neutral

    // ---------------- blended pixel ----------------
    output reg  [23:0] vout_rgb
);

    // ==================================================================
    //  stage 1 : transition blend  (base vs scaled)
    //      out = (bg * (256 - a) + fg * a) >> 8
    //  Applied per channel.
    //
    //  End-point exactness: the plain >>8 form gives
    //      a=0   -> bg + 0      (exact, because bg*256 >> 8 == bg)
    //      a=255 -> fg - 1      (ONE LSB LOW)
    //  A one-LSB error is invisible, but a hard cut that lands on 254 instead
    //  of 255 is the kind of thing that shows up as a faint flicker on some
    //  panels, so the two end points are handled explicitly. Cost is a pair of
    //  comparators per channel -- a few LUTs, no extra multiplier.
    // ==================================================================
    wire [8:0] ta = {1'b0, trans_alpha};        // 0..255 in 9 bits

    function [7:0] blend8;
        input [7:0] bg;
        input [7:0] fg;
        input [8:0] a;                          // 0..255
        reg  [15:0] bg_w;
        reg  [15:0] fg_w;
        reg  [16:0] sum;
        begin
            if (a == 9'd0)          blend8 = bg;        // exact
            else if (a == 9'd255)   blend8 = fg;        // exact
            else begin
                bg_w = bg << 8;                         // bg * 256, no truncation
                bg_w = bg_w - (bg * a);                 // bg * (256 - a)
                fg_w = fg * a;
                sum  = {1'b0, bg_w} + {1'b0, fg_w};     // <= 130305, 17 bits
                blend8 = sum[15:8];
            end
        end
    endfunction

    wire [7:0] t_r = blend8(base_rgb[23:16], scale_rgb[23:16], ta);
    wire [7:0] t_g = blend8(base_rgb[15:8],  scale_rgb[15:8],  ta);
    wire [7:0] t_b = blend8(base_rgb[7:0],   scale_rgb[7:0],   ta);

    wire [23:0] trans_rgb = {t_r, t_g, t_b};

    // ==================================================================
    //  stage 2 : text overlay
    //  text_alpha is binary in this design, so it is a multiplexer.
    //  The multiply form is kept commented for reference.
    // ==================================================================
    wire text_opaque = (text_alpha == 8'd255);

    wire [23:0] text_rgb_mux = text_opaque ? text_rgb : trans_rgb;
    // alternative (smooth alpha):
    //   {blend8(trans_rgb[23:16], text_rgb[23:16], text_alpha[8:0]), ...}

    wire [23:0] after_text = USE_TEXT ? text_rgb_mux : trans_rgb;

    // ==================================================================
    //  stage 3 : spectrum overlay (from person C)
    // ==================================================================
    wire spec_in_area = (x >= SPEC_X[9:0]) && (x < (SPEC_X[9:0] + SPEC_W[9:0])) &&
                        (y >= SPEC_Y[9:0]) && (y < (SPEC_Y[9:0] + SPEC_H[9:0]));

    wire spec_opaque = (spec_alpha == 8'd255) && spec_in_area;

    wire [23:0] after_spec = (USE_SPEC && spec_opaque) ? spec_rgb : after_text;

    // ==================================================================
    //  stage 4 : brightness and contrast, applied last
    //      y = (v * contrast) >> 7          contrast 128 -> x1.0
    //      out = saturate(y + brightness - 128)   brightness 128 -> no offset
    //  Everything stays unsigned 8-bit; the offset is done with a
    //  pre-clamped 9-bit unsigned add so there is no signed-arithmetic trap.
    // ==================================================================
    function [7:0] enhance;
        input [7:0] v;
        input [7:0] c;
        input [7:0] b;
        reg   [15:0] m;
        reg   [8:0]  y;
        reg   [9:0]  s;
        begin
            m = v * c;                      // 255*255 = 65025 -> 16 bits
            // NINE BITS, NOT EIGHT. `y` is declared [8:0] for a reason: with
            // c > 128 the product (v*c)>>7 reaches 508, which does not fit in
            // 8 bits.
            //
            // This read `m[14:7]`, which is only 8 bits wide, so the value was
            // silently reduced modulo 256 and stored into the 9-bit y. The
            // symptom is not a small error -- it is NON-MONOTONIC:
            //
            //     v=129, c=255  ->  (v*c)>>7 = 256  -> should clamp to 255 (white)
            //                       m[14:7]  = 0    -> actually renders BLACK
            //
            // so a high contrast setting inverted highlights instead of
            // brightening them (at contrast 255, 128->255 but 129->0).
            //
            // NOT REACHABLE IN THIS BITSTREAM: top_tf_hdmi_audio.v ties
            // `.param_contrast (8'd128)`, and v*128 = v<<7 has bit 15 clear, so
            // m[15:7] == m[14:7] for all 256 values of v. The picture today is
            // bit-for-bit unchanged by this fix. It becomes visible the moment
            // person B drives a real contrast value, which is exactly what the
            // parameter bank is for.
            y = m[15:7];                    // (v*c) >> 7, range 0..508
            if (b >= 8'd128)
                s = {1'b0, y} + {1'b0, (b - 8'd128)};        // shift up
            else if (y > {1'b0, (8'd128 - b)})
                s = {1'b0, y} - {1'b0, (8'd128 - b)};        // shift down
            else
                s = 10'd0;                                   // clamped at 0
            if (s > 10'd255) enhance = 8'd255;
            else             enhance = s[7:0];
        end
    endfunction

    wire [7:0] e_r = enhance(after_spec[23:16], contrast, brightness);
    wire [7:0] e_g = enhance(after_spec[15:8],  contrast, brightness);
    wire [7:0] e_b = enhance(after_spec[7:0],   contrast, brightness);

    wire [23:0] enhanced = USE_ENHANCE ? {e_r, e_g, e_b} : after_spec;

    // ==================================================================
    //  stage 5 : transition dimming  (fade to black and back)
    //
    //  WHY THIS STAGE EXISTS AT ALL
    //  ----------------------------
    //  trans_alpha was originally wired straight into the crossfade at the top
    //  of this module. That crossfade blends base_rgb against scale_rgb, but in
    //  this project BOTH inputs come from the same SDRAM display buffer
    //  (vout_data_raw), so the result was
    //        out = img*(1-a) + img*a  ==  img
    //  for every value of a. A "fade" that is mathematically the identity is
    //  invisible, which is exactly what was observed on the board.
    //
    //  A real dissolve would need the OLD and the NEW frame at the same time.
    //  The SDRAM read port has a single read_addr_index, so only one frame is
    //  reachable and a dissolve is not possible without reworking the memory
    //  path. A fade through black needs only the current frame:
    //
    //        out = img * trans_alpha / 255
    //
    //  trans_alpha = 255 -> unchanged (normal display)
    //  trans_alpha = 0   -> black (the cut point, where the buffer swaps)
    //  so the sequence reads as: darken, swap, brighten. That IS a visible
    //  transition, which is what the contest extension asks for.
    //
    //  PLACEMENT: between the enhancer and the OSD, i.e. BELOW the OSD.
    //  The OSD shows the parameter bar, and it must stay readable while the
    //  picture darkens -- dimming the operator's own control would be wrong.
    //  Dimming is applied to `enhanced` and the OSD is multiplexed on top of
    //  the dimmed value, so a lit OSD pixel bypasses the dimmer entirely.
    //
    //  COST: one 8x8 multiply per channel. The two end points are handled
    //  explicitly so a=0 gives an exact 0 and a=255 returns the input exactly
    //  (the plain >>8 form would return v-1 at the top end, the same one-LSB
    //  artefact the crossfade stage documents above).
    // ==================================================================
    function [7:0] dim8;
        input [7:0] v;
        input [8:0] a;                  // 0..255
        reg  [15:0] m;
        begin
            if (a == 9'd0)        dim8 = 8'd0;
            else if (a >= 9'd255) dim8 = v;
            else begin
                m = v * a[7:0];
                dim8 = m[15:8];
            end
        end
    endfunction

    wire [7:0] d_r = dim8(enhanced[23:16], ta);
    wire [7:0] d_g = dim8(enhanced[15:8],  ta);
    wire [7:0] d_b = dim8(enhanced[7:0],   ta);

    wire [23:0] dimmed = USE_FADE ? {d_r, d_g, d_b} : enhanced;

    // ==================================================================
    //  stage 6 : radial iris  (the picture closes in from the edges)
    //
    //  THIS IS NOT THE SAME EFFECT AS THE DIMMER ABOVE, and confusing the two
    //  is exactly what happened once already:
    //
    //    dimmer (stage 5)  multiplies the WHOLE picture by one number. Every
    //                      pixel changes together, so the screen just gets
    //                      darker and then brighter -- "a flash".
    //    iris   (this one) is a SPATIAL mask. Pixels far from the centre go
    //                      black first and the dark region sweeps inwards, so
    //                      the visible area shrinks like a closing pupil.
    //
    //  The request was the iris, so this stage is what actually delivers the
    //  wanted look; the dimmer stays because it is harmless and useful.
    //
    //  GEOMETRY: a diamond. A pixel at (x,y) offset from the centre by
    //  (dx,dy) is inside the aperture when
    //        |dx| * (V_ACTIVE/2) + |dy| * (H_ACTIVE/2)  <  aperture * K
    //  with K = (H_ACTIVE/2) * (V_ACTIVE/2). Written that way each ordinary
    //  pixel costs one 10x10 multiply plus one 10x11 multiply, with no square
    //  root and no divider anywhere.
    //
    //  A diamond rather than a circle on purpose: a true circle needs a square
    //  root per pixel, and a diamond is the standard cheap approximation used
    //  for wipes. Swapping in a circle later only changes this comparison.
    //
    //  TIMING: iris_en is high during the swap and for IRIS_LEN clocks after
    //  it, so the effect plays entirely on the NEW picture, revealing it from
    //  the centre outwards.
    //
    //  WHAT THIS CANNOT DO. A closing iris over the OLD picture is impossible
    //  here: the SDRAM read port has a single address index, so the moment the
    //  buffer swaps the old frame is no longer reachable. The old picture can
    //  therefore never be shown shrinking -- only the new one can be revealed.
    // ==================================================================
    // ------------------------------------------------------------------
    //  IRIS DIMENSIONS -- verified with iris_check.py BEFORE writing this.
    //
    //      iris_d_metric = |dx|*(V/2) + |dy|*(H/2)
    //
    //  For 640x480 that is |x-320|*240 + |y-240|*320, which runs
    //      0        at the exact centre (exactly one pixel has 0)
    //      153600   at a corner, the maximum over the whole frame
    //
    //  The aperture arrives from the caller IN THOSE SAME UNITS, so the only
    //  operation here is a compare -- no multiply, no divide, nothing to
    //  mis-scale.
    //
    //  History, because it is easy to repeat: three earlier attempts failed on
    //  dimension errors. One multiplied a 0..1024 progress value by the wrong
    //  constant and made the aperture ~256x too large (mask invisible); the
    //  next left the corners black forever. Both were arithmetic mistakes that
    //  looked fine in the code. Hence: one unit, one place, compare only.
    // ------------------------------------------------------------------
    //  The metric is NOT symmetric in the two halves: dx (the x offset) is
    //  weighted by V/2 = 240 and dy (the y offset) by H/2 = 320, so a corner
    //  reaches dx*240 + dy*320 = 320*240 + 240*320 = 153600. That is the
    //  number the aperture must exceed, and it is twice (H/2)*(V/2) = 76800,
    //  NOT (H/2)*(V/2) + 1 as an earlier version of this line assumed -- with
    //  DMAX = 76801 the corners stayed black forever.
    //
    //  The +1 is only a strictness margin: a pixel is masked when
    //  iris_d >= aperture, so the aperture must reach one past the largest
    //  metric.
    localparam [31:0] IRIS_DMAX = ((H_ACTIVE / 2) * V_ACTIVE) + 1;

    // Expose the parameter arithmetic so a testbench can READ the computed
    // values instead of re-deriving them. Reasoning about what a Verilog
    // expression evaluates to got this wrong more than once; printing it does
    // not.
    wire [31:0] dbg_iris_dmax = IRIS_DMAX;
    wire [31:0] dbg_half_h    = H_ACTIVE / 2;
    wire [31:0] dbg_half_v    = V_ACTIVE / 2;

    wire [9:0]  cx  = H_ACTIVE[9:0] >> 1;      // 320 for 640
    wire [9:0]  cy  = V_ACTIVE[9:0] >> 1;      // 240 for 480

    wire [10:0] dxr = (x > cx) ? ({1'b0, x} - {1'b0, cx}) : ({1'b0, cx} - {1'b0, x});
    wire [10:0] dyr = (y > cy) ? ({1'b0, y} - {1'b0, cy}) : ({1'b0, cy} - {1'b0, y});

    wire [31:0] iris_d_metric = (dxr * (V_ACTIVE[9:0] >> 1))
                              + (dyr * (H_ACTIVE[9:0] >> 1));

    // when the iris is off, hold the aperture fully open so the mask cannot
    // touch anything -- a stuck iris must never be able to blank the screen
    wire [31:0] iris_open = iris_en ? iris_aperture : IRIS_DMAX;

    wire iris_out = (iris_d_metric >= iris_open);   // outside -> black

    wire [23:0] after_iris = (USE_IRIS && iris_out) ? 24'd0 : dimmed;

    // ==================================================================
    //  stage 7 : OSD overlay  (person B: the parameter adjustment bar)
    //
    //  Sits ABOVE the dimmer and the iris on purpose: the OSD shows the very
    //  parameters being adjusted, so it must not darken or shrink along with
    //  the picture.
    //
    //  Like the text layer, its alpha is binary in this design, so it is a
    //  multiplexer -- no extra multiplier.
    // ==================================================================
    wire osd_opaque = (osd_alpha == 8'd255);

    wire [23:0] after_dim = (USE_OSD && osd_opaque) ? osd_rgb : after_iris;

    // ==================================================================
    //  output register
    //  bypass is evaluated here so the caller can switch the whole layer
    //  stack off without touching the data path
    // ==================================================================
    always @(posedge clk or posedge rst) begin
        if (rst)
            vout_rgb <= 24'd0;
        else if (bypass)
            vout_rgb <= base_rgb;
        else
            vout_rgb <= after_dim;
    end

endmodule
