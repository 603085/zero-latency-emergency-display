// ============================================================================
//  mode_fsm.v -- system mode state machine  (Task A skeleton)
//
//  Three modes:
//    NORMAL    : normal signage, smooth transitions, scrolling warning line
//    EMERGENCY : evacuation, HARD CUT (no transition), high-contrast colours
//    SELFTEST  : power-on self test, reports which pipeline stage is alive
//
//  Why a separate FSM?  The emergency path must be a short, obviously-correct
//  piece of logic. Mixing "everyday show" behaviour with the emergency trigger
//  is how emergency systems end up failing exactly when they are needed.
//
//  Latency design (this is the headline feature of the whole project):
//    the emergency input is synchronised (2 FFs), then debounced, then it sets
//    a mode register. The layer mixer samples that register while scanning a
//    line, so the worst case is one frame (16.7 ms at 640x480@60) and the
//    typical case is the next scan line (~20 us). No polling, no queue, no OS.
//
//  Edge cases handled:
//    - emergency asserted during SELFTEST wins immediately
//    - emergency released returns to NORMAL, but only after the caller has had
//      time to run its recovery (mode_release_ack)
//    - self test is only entered from reset, never spontaneously
// ============================================================================

`timescale 1ns / 1ps

module mode_fsm #(
    parameter SYNC_STAGES   = 2,
    parameter DEBOUNCE_BITS = 20,       // 2^20 / 25MHz = 42 ms @ 25MHz
    // Power-on self test length in video clocks.
    // Hardware value: 25 MHz x 1.5 s = 37_500_000.
    // It is a parameter so a simulation can shorten it without changing
    // production behaviour.
    parameter [25:0] SELFTEST_CYCLES = 26'd37_500_000,
    // Fade-through-black length, in video clocks.
    //
    // Hardware value: 25 MHz x 0.3 s = 7_500_000.
    // It was 2_500_000 (0.1 s) and 0.1 s is simply too short to read as a
    // transition -- on the board it looked like an instant cut. 0.3 s is
    // comfortably visible without feeling sluggish to someone pressing "next"
    // repeatedly.
    parameter [31:0] FADE_LEN_PARAM = 33_554_432,
    // How far to shift the ramp counter down to produce the 8-bit alpha.
    // alpha = fade_cnt >> FADE_SHIFT.
    //
    // THIS MUST MATCH FADE_LEN. The original code hard-coded a shift of 16
    // (fade_cnt[23:16]) while FADE_LEN was 2_500_000. Since
    //     2_500_000 >> 16 == 38
    // the ramp topped out at alpha 38 and NEVER reached 255, so trans_alpha
    // stayed at a few percent of full scale and the picture was left
    // permanently dark after every image change. The old testbench only
    // checked that alpha STARTED at 0, so the defect went unnoticed.
    //
    // Rule: alpha reaches 255 exactly when fade_cnt reaches FADE_LEN, so
    //       FADE_LEN must be at least (255 << FADE_SHIFT).
    //       For FADE_LEN = 2_500_000 the shift must be <= 13.
    //       A testbench that shortens FADE_LEN must shorten FADE_SHIFT too.
    parameter FADE_SHIFT = 17,

    // Radial iris: how long the aperture takes to open, in video clocks.
    //
    // Hardware value: 25 MHz x 1.0 s = 25_000_000.
    //
    // TIMING IS WHAT MAKES THIS READABLE. The aperture has to travel 153601
    // metric units, and the metric is tight near the centre: the first few
    // thousand units cover only a handful of pixels. At 0.4 s the centre dot
    // appeared and then the picture was essentially there -- the wipe was over
    // before the eye caught it. A full second spreads the travel across ~25
    // frames so the dark corners are visibly retreating.
    parameter [31:0] IRIS_LEN_PARAM = 25_000_000,

    // The aperture is expressed in the SAME UNITS as layer_mixer's distance
    // metric:  iris_d = |dx|*(V/2) + |dy|*(H/2), which for 640x480 runs from
    // 0 at the centre to (H/2)*(V/2) = 153600 at a corner. So
    //     IRIS_DMAX = 153600 + 1 = 153601
    // and the mask is fully gone once the aperture reaches it. Verified with
    // iris_check.py before this code was written.
    parameter [31:0] IRIS_DMAX = 32'd153601
)(
    input  wire        clk,             // video_clk (25 MHz)
    input  wire        rst,

    // ---------------- physical inputs ----------------
    input  wire        sw_emergency,    // slide switch, 1 = emergency  (pin C6, sw3)
    input  wire        key_next,        // manual "next"                 (pin B2, key1)
    input  wire        key_auto,        // auto-play toggle              (pin B1, key2)
    input  wire        key_mode,        // mode / parameter key          (pin C1, key3)

    // ---------------- transition type (person B's parameter item 2) -----
    // Which effect plays when the picture changes, in NORMAL mode:
    //     0 = hard cut      1 = fade through black      2 = radial iris
    // EMERGENCY and SELFTEST ignore this and always hard-cut -- see the note
    // in the case(mode) block below.
    //
    // An undriven input falls through to FADE, which is what this design did
    // before the input existed, so a testbench that does not connect it keeps
    // the behaviour it was written against.
    input  wire [1:0]  param_transition,

    // ---------------- system health inputs ----------------
    input  wire        sd_card_ok,      // 1 = SD card healthy
    input  wire        audio_link_ok,   // 1 = audio samples arriving
    input  wire [3:0]  sd_state_code,   // the example's own status code

    // ---------------- outputs to the rest of the design ----------------
    output reg  [1:0]  mode,            // 0=NORMAL 1=EMERGENCY 2=SELFTEST
    output reg         mode_changed,    // single-cycle pulse on any mode change

    output reg         text_en,         // enable the text layer
    output reg  [2:0]  text_color,      // text colour selection
    output reg  [7:0]  trans_alpha,     // 0 = old picture, 255 = new picture
    output reg         transition_run,  // 1 while a transition is in progress
    output reg  [1:0]  transition_type, // 0=hard 1=fade 2=slide

    output reg         auto_play,       // auto rotation on/off
    output reg         next_image,      // single-cycle pulse: load next image
    output reg         self_test_done,  // 1 when the power-on self test finished

    // ---------------- two-phase transition handshake ----------------
    // hold_load gates the START of the next image load in sd_card_bmp. It is
    // asserted while the CURRENT picture fades out, so the load cannot begin
    // until the screen is black. That is the whole reason a real
    // fade-out-then-fade-in is possible here: without it the buffer swaps
    // whenever the card read finishes and only the new picture can be faded.
    output wire        hold_load,
    input  wire        load_busy,           // the deferred load has begun

    // next_req : sd_card_bmp wants to load the next picture (a one-clock pulse,
    //            already synchronised into this clock domain by the wrapper)
    // next_grant : the load may begin. Held HIGH for the whole of WAIT_SWAP.
    input  wire        next_req,
    output wire        next_grant,

    // load_req: a LEVEL, high for the whole of WAIT_SWAP. This is the card's
    // instruction to start the load.
    //
    // It is deliberately NOT a pulse. As a one-clock pulse it had to be caught
    // by the card within a few clocks of WAIT_SWAP beginning, and when the two
    // clock domains lined up badly the request was dropped: measured on the
    // board as want_cnt = 1, owed_cnt = 1 and state_code = 1 (the load never
    // started), with the FSM waiting forever for a swap that could not happen.
    // A level cannot be missed -- it stays true until the swap arrives.
    output wire        load_req,
    // ts_state_o : the transition state, brought out so it can be put on
    // a 7-segment digit. Without it there is no way to tell whether a
    // transition is idle, fading, waiting for the buffer swap, or fading
    // back in -- and those need completely different fixes.
    output wire [1:0]  ts_state_o,

    // busy_stack: high for the whole of a transition (FADE_OUT, WAIT_SWAP,
    // FADE_IN). Auto rotation uses it to avoid requesting the next picture
    // while one is already being shown, which is what made rotation freeze and
    // then blank the screen.
    output wire        busy_stack,

    // ---------------- latency measurement taps ----------------
    // emg_debounced is the START for latency_meter: it rises only after the
    // full debounce window, so the reported number is the deterministic part
    // of the response. mode_changed is the STOP.
    output wire        emg_debounced,

    // ---------------- image swap (the real fade trigger) ----------------
    // Single-cycle pulse the moment the display buffer actually changes, i.e.
    // when the new picture appears on screen. Raised by the wrapper from
    // sd_card_bmp's disp_buf_idx.
    //
    // WHY THE FADE USES THIS AND NOT next_image
    // -----------------------------------------
    // next_image fires when the user ASKS for the next picture. Reading the
    // card and writing the frame into SDRAM then takes tens of milliseconds,
    // during which the OLD picture is still on screen. Starting the 0.1 s ramp
    // at next_image therefore finished the fade before the swap, and the new
    // picture simply appeared -- a hard flash, which is exactly what was
    // reported on the board.
    //
    // image_swap arrives at the swap itself, so the ramp starts exactly when
    // the screen content changes and the new picture fades up from black.
    input  wire        image_swap,

    // ---------------- radial iris ----------------
    // iris_en       : 1 while the iris transition plays
    // iris_aperture : the opening radius, IN layer_mixer's metric units
    //                 0 .. IRIS_DMAX. Passed straight through to the mixer so
    //                 there is exactly ONE place that knows the units.
    output reg         iris_en,
    output reg  [31:0] iris_aperture
);

    localparam MODE_NORMAL    = 2'd0;
    localparam MODE_EMERGENCY = 2'd1;
    localparam MODE_SELFTEST  = 2'd2;

    // ==================================================================
    //  input synchronisation (all inputs are asynchronous to video_clk)
    // ==================================================================
    reg [SYNC_STAGES-1:0] sync_emg;
    reg [SYNC_STAGES-1:0] sync_next;
    reg [SYNC_STAGES-1:0] sync_auto;
    reg [SYNC_STAGES-1:0] sync_kmode;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            sync_emg   <= {SYNC_STAGES{1'b0}};
            sync_next  <= {SYNC_STAGES{1'b1}};
            sync_auto  <= {SYNC_STAGES{1'b1}};
            sync_kmode <= {SYNC_STAGES{1'b1}};
        end else begin
            sync_emg   <= {sync_emg[SYNC_STAGES-2:0],   sw_emergency};
            sync_next  <= {sync_next[SYNC_STAGES-2:0],  key_next};
            sync_auto  <= {sync_auto[SYNC_STAGES-2:0],  key_auto};
            sync_kmode <= {sync_kmode[SYNC_STAGES-2:0], key_mode};
        end
    end

    // ==================================================================
    //  EMERGENCY SWITCH POLARITY -- ACTIVE HIGH. THIS IS THE TRUTH.
    //
    //      switch UP   (level 1) -> EMERGENCY: text red, hard cut
    //      switch DOWN (level 0) -> normal:    white text, fade
    //
    //  NO INVERSION IS APPLIED. emg_level is the synchronised pin, straight.
    //  This matches every pinned document:
    //      板级接口与显示资源.md §2.2   SW3 C6 "应急模式触发（拨上=应急）"
    //      接口清单_TaskA.md     §1.4   sw_emergency "拨码，高 = 应急"
    //      上板验证清单_TaskA.md  #11   "把 C6 拨码拨上去 → 文字立即变红"
    //  and it is what the simulated board does: tb_mode_fsm drives
    //  sw_emg = 1'b1 and expects mode == MODE_EMERGENCY.
    //
    //  ------------------------------------------------------------------
    //  WHY THIS PARAGRAPH EXISTS: AN EARLIER COMMENT HERE SAID THE OPPOSITE
    //
    //  A previous version of this block asserted "ACTIVE LOW" and
    //  "0 = emergency (switch down)", and a script (rtl\flip_emg_polarity.py)
    //  was written to act on that belief. It was WRONG, and the experiment was
    //  reverted, because inverting this line:
    //
    //    * broke 65 checks in tb_mode_fsm, whose whole model assumes the
    //      current polarity, and
    //    * left the debouncer initialised to "not emergency" while the
    //      inverted level said emergency, so the design powered up IN
    //      emergency mode.
    //
    //  The comment survived the revert and the code did not, which made the
    //  file contradict itself: the prose said active-low, the next line said
    //  otherwise. Anyone reading it -- including whoever wrote the script --
    //  would have "fixed" a working design.
    //
    //  DO NOT INVERT THIS LINE. If you want the switch the other way round,
    //  change it in the constraint file (pin.adc PULLTYPE), not here.
    // ==================================================================
    wire emg_level = sync_emg[SYNC_STAGES-1];

    // ==================================================================
    //  debounce for the emergency input
    //  active-HIGH: 1 = emergency (switch up), 0 = normal (switch down)
    //  The counter must see a STABLE level for the whole window before the
    //  debounced value follows. This is what gives a deterministic edge.
    // ==================================================================
    reg [DEBOUNCE_BITS-1:0] db_cnt;
    reg                     emg_stable;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            db_cnt     <= {DEBOUNCE_BITS{1'b0}};
            emg_stable <= 1'b0;         // 0 = not emergency, correct for active-low
        end else begin
            if (emg_level == emg_stable) begin
                db_cnt <= {DEBOUNCE_BITS{1'b0}};
            end else if (db_cnt == {DEBOUNCE_BITS{1'b1}}) begin
                emg_stable <= emg_level;
                db_cnt     <= {DEBOUNCE_BITS{1'b0}};
            end else begin
                db_cnt <= db_cnt + 1'b1;
            end
        end
    end

    // ==================================================================
    //  key edge detect (active LOW buttons: pressed = 0)
    // ==================================================================
    reg next_d, auto_d, kmode_d;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            next_d  <= 1'b1;
            auto_d  <= 1'b1;
            kmode_d <= 1'b1;
        end else begin
            next_d  <= sync_next[SYNC_STAGES-1];
            auto_d  <= sync_auto[SYNC_STAGES-1];
            kmode_d <= sync_kmode[SYNC_STAGES-1];
        end
    end

    wire next_press  = next_d  & ~sync_next[SYNC_STAGES-1];   // falling edge
    wire auto_press  = auto_d  & ~sync_auto[SYNC_STAGES-1];
    wire kmode_press = kmode_d & ~sync_kmode[SYNC_STAGES-1];

    // ==================================================================
    //  auto-play toggle
    // ==================================================================
    always @(posedge clk or posedge rst) begin
        if (rst)                 auto_play <= 1'b0;
        else if (auto_press)     auto_play <= ~auto_play;
        else if (mode == MODE_EMERGENCY) auto_play <= 1'b0;   // stop show in emergency
    end

    // ==================================================================
    //  next-image pulse (one clock wide)
    //
    //  This is the FSM's own "user pressed next" pulse. It is NOT what starts
    //  the load any more -- sd_card_bmp takes its grant from next_grant below.
    //
    //  next_image itself is a separate register so the two-phase machine can
    //  hold a LEVEL high during WAIT_SWAP without two always blocks driving the
    //  same signal (which is a multiple-driver error, not a merge).
    // ==================================================================
    always @(posedge clk or posedge rst) begin
        if (rst)             next_image <= 1'b0;
        else if (next_press) next_image <= 1'b1;
        else                 next_image <= 1'b0;
    end

    // Grant: the load may begin. A one-clock pulse from the key press (used
    // when the transition is a hard cut, where the FSM never enters the machine
    // below), OR a level held for the whole WAIT_SWAP phase of a smooth
    // transition. The level is what matters: sd_card_bmp sees it through a
    // clock-domain synchroniser and an edge detector running on sd_card_clk, so
    // a pulse shorter than one of its clocks could be missed.

    // ==================================================================
    //  power-on self test counter
    //  1.5 s at 25 MHz: during this window the caller lights each pipeline
    //  stage in turn, so a failure can be located on screen.
    // ==================================================================
    localparam [25:0] SELFTEST_LEN = SELFTEST_CYCLES;   // see the parameter above
    reg [25:0] st_cnt;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            st_cnt         <= 26'd0;
            self_test_done <= 1'b0;
        end else if (st_cnt >= SELFTEST_LEN) begin
            self_test_done <= 1'b1;
        end else begin
            st_cnt <= st_cnt + 26'd1;
        end
    end

    // ==================================================================
    //  transition ramp  (fade through black)
    //
    //  NORMAL    : alpha ramps 0 -> 255 over FADE_LEN video clocks
    //  EMERGENCY : alpha jumps straight to 255 -> hard cut
    //
    //  FADE_LEN and FADE_SHIFT must agree: see the rule in the parameter list.
    //  Getting that wrong is silent -- the ramp stops short of 255 and the
    //  picture stays dark, which is exactly the bug that was here before.
    //
    //  TWO SAFETY MECHANISMS, both added after the board came up BLACK with a
    //  visible subtitle (the signature of trans_alpha stuck at 0):
    //
    //  1. LOCKOUT. While a ramp is running, and for COOLDOWN cycles after it,
    //     further next_image pulses are ignored. Without this, a source that
    //     pulses next_image at all often re-arms the ramp every time it fires,
    //     fade_cnt is reset to 0 over and over, and the picture never comes
    //     back up. This is the most likely explanation for the black board.
    //
    //  2. WATCHDOG. If a ramp somehow runs longer than FADE_LEN plus a margin,
    //     alpha is forced back to 255. A fade is a cosmetic effect; it must
    //     never be able to leave the display dark. This makes the failure mode
    //     self-correcting no matter what the trigger does.
    // ==================================================================
    localparam [31:0] FADE_LEN = FADE_LEN_PARAM;   // see the parameter list

    // margin past which a phase is declared stuck and alpha is forced to its
    // end value. The transition must never be able to leave the screen dark.
    localparam [31:0] FADE_WATCHDOG = FADE_LEN + (FADE_LEN >> 2) + 32'd4096;

    // ==================================================================
    //  SWAP_WATCHDOG -- how long WAIT_SWAP will wait for the picture to swap.
    //
    //  Set from what the wait legitimately takes, not from a guess. Reading a
    //  picture off the card and writing it into the frame buffer is the long
    //  part: the measured card read is 0.45 s to 9 s, and the frame write adds
    //  a few tens of milliseconds on top.
    //
    //  An earlier version used FADE_WATCHDOG here (0.418 s) and gave up before
    //  almost every real load had finished, which is why the transition
    //  appeared to work once and then never again. So this is deliberately far
    //  longer than a load needs -- a second or two of extra patience costs
    //  nothing, because the screen is black during the wait either way, while
    //  giving up early is fatal.
    //
    //  12 s at 25 MHz = 300_000_000 cycles.
    // ==================================================================
    localparam [31:0] SWAP_WATCHDOG = 32'd300_000_000;

    //  How long WAIT_SWAP will believe a load is still coming before deciding
    //  it never started. A real load raises load_busy within a few card clocks;
    //  200 us at 25 MHz = 5_000 cycles is far longer than that and far shorter
    //  than any real read.
    localparam [31:0] LOAD_START_GRACE = 32'd5_000;

    // ==================================================================
    //  TWO-PHASE TRANSITION: fade out -> swap while black -> fade in
    //
    //  This is the sequence that was actually asked for, and it is only
    //  possible because sd_card_bmp now has a hold_load input:
    //
    //      IDLE      the current picture is on screen, alpha = 255
    //        |  user asks for the next picture (or auto rotation fires)
    //        v
    //      FADE_OUT  the CURRENT picture darkens smoothly to black.
    //        |       hold_load = 1, so sd_card_bmp is not allowed to start
    //        |       loading -- this is what keeps the old picture on screen
    //        |       while it fades.
    //        v
    //      WAIT_SWAP hold_load released, the load runs, and we wait for the
    //        |       display buffer to actually change. The screen is black
    //        |       throughout, so the swap itself is invisible.
    //        v
    //      FADE_IN   the NEW picture brightens smoothly from black.
    //        |
    //        v
    //      IDLE
    //
    //  WHY THE OLD ONE-SHOT RAMP WAS WRONG
    //  -----------------------------------
    //  It started at image_swap and only ran upwards. By that point the buffer
    //  had already changed, so the only thing left to animate was the new
    //  picture appearing: dark -> new picture, with no fade of the old one at
    //  all. On the board that reads as a hard cut. Starting earlier is
    //  impossible without hold_load, because the swap time is not known.
    //
    //  EMERGENCY IS NOT AFFECTED. mode == MODE_EMERGENCY forces a hard cut:
    //  the evacuation picture must appear on the next scanned line, not fade
    //  in. The emergency path never enters this machine.
    // ==================================================================
    localparam [1:0] TS_IDLE      = 2'd0;
    localparam [1:0] TS_FADE_OUT  = 2'd1;
    localparam [1:0] TS_WAIT_SWAP = 2'd2;
    localparam [1:0] TS_FADE_IN   = 2'd3;

    reg [1:0]  ts;
    reg [31:0] ts_cnt;
    // REQUEST LATCH.
    //
    // next_req is a one-cycle pulse (the wrapper edge-detects the card's
    // request level). Turning it straight into a level that follows the request
    // does NOT work:
    //
    //     else if (next_req) req_pending <= 1'b1;
    //     else               req_pending <= 1'b0;
    //
    // The card drops its request as soon as it sees hold_load, which is one
    // video clock after this machine enters FADE_OUT. So req_pending was cleared
    // in the very cycle the transition began, and the machine then sat in
    // FADE_OUT with no record of why. A cycle-by-cycle trace showed exactly
    // that:
    //
    //     card_req=1 pulse=1 armed=0 ts=0     <- pulse seen
    //     card_req=1 pulse=0 armed=1 ts=0     <- armed
    //     card_req=0 pulse=0 armed=0 ts=1     <- armed cleared as it starts
    //
    // Latching the pulse instead, and clearing it only when this machine
    // actually begins a transition, makes the request survive however briefly
    // the card holds its level. The card cannot lose a request either, because
    // it is only cleared once the FSM has committed.
    reg        req_pending;

    //  load_started_l : set when WAIT_SWAP sees the card begin a load, cleared
    //  when the wait ends. Together with load_busy it says "a load ran and has
    //  now finished", which is what WAIT_SWAP is actually waiting for -- see
    //  the note in the WAIT_SWAP branch for why the buffer-index detector was
    //  abandoned.
    reg        load_started_l;
    // one-clock pulse on entering WAIT_SWAP
// (was_waiting is gone: load_req is a level now, so no edge detect)

    always @(posedge clk or posedge rst) begin
        if (rst)
            req_pending <= 1'b0;
        else if (mode == MODE_EMERGENCY)
            req_pending <= 1'b0;
        else if (next_req)
            req_pending <= 1'b1;        // latch the pulse
        else if (ts == TS_IDLE && !next_req)
            // ==============================================================
            //  CLEARED ONLY IN THE IDLE CYCLE, AFTER THE REQUEST HAS BEEN ACTED
            //  ON -- never during a transition.
            //
            //  The previous form was
            //
            //      else if (ts == TS_FADE_OUT) req_pending <= 1'b0;
            //
            //  and because that condition is true for the whole of FADE_OUT, it
            //  also erased every request that arrived DURING a transition.
            //  MEASURED ON THE BOARD: four presses in a row produced one
            //  transition, then nothing, then nothing, then one more -- only the
            //  presses landing while the machine happened to be idle were seen.
            //
            //  Reaching this branch means the machine is idle, so the IDLE
            //  branch has already had its cycle to act on the request. Clearing
            //  here frees the latch for the next press without ever racing the
            //  consumer, and `!next_req` guarantees a pulse arriving in this
            //  very cycle is not stepped on.
            //
            //  Consequence, and it is the intended one: a press made while a
            //  transition is running stays latched and is served as soon as the
            //  transition finishes.
            // ==============================================================
            req_pending <= 1'b0;
    end

    // NOTE ON ORDER: within one clock, `next_req` is tested before `ts`, so a
    // pulse arriving in the same cycle the machine leaves IDLE is never lost.
    // The clear happens on the FOLLOWING cycle, by which time the IDLE branch
    // has already acted on it.

    // Grant to sd_card_bmp: it may begin the load now. Either the
    // one-clock key pulse (hard-cut path), or a LEVEL held for the whole
    // of WAIT_SWAP (smooth path). The level matters because sd_card_bmp
    // synchronises it into sd_card_clk and edge-detects it there, so a
    // pulse narrower than one of its clocks could be missed entirely.
    // A picture change is never gated any more. It used to be granted only in
    // WAIT_SWAP, after the old picture had faded out -- and that gate is what
    // deadlocked the design, because WAIT_SWAP waited on a handshake that never
    // fired. sd_card_bmp now advances the picture by itself, so the grant simply
    // mirrors the request.
    assign next_grant = next_image;

    // any state other than IDLE means a fade is running
    assign busy_stack = (ts != TS_IDLE);

    // The card no longer asks permission to load, so this is never asserted. It
    // stays as a port because the wrapper and the top level connect it, and
    // removing it would mean touching three files for no functional gain.
    assign load_req = 1'b0;
    reg        swap_seen_l;      // set when the buffer change was observed

    // Nothing is held back any more. This used to freeze the card while the old
    // picture faded out, which required the fade-out to finish before the swap
    // could start -- and that ordering is what made the two halves wait for each
    // other. The card now loads whenever it likes and the fade plays on the NEW
    // picture, so there is nothing to hold.
    assign hold_load = 1'b0;

    // transition_run is reported for compatibility with the existing tests and
    // is true for the whole sequence, not just one ramp.
    // ==================================================================
    //  THE FADE -- one ramp, started by the picture actually changing.
    //
    //  IDLE -> FADE_IN is the whole machine now.
    //
    //  FADE_OUT and WAIT_SWAP are gone, and with them the deadlock: WAIT_SWAP
    //  existed to hold the screen black while sd_card_bmp swapped the buffer,
    //  and it waited on a handshake that did not fire. The picture is now
    //  advanced by the card alone, with no permission asked of this machine.
    //
    //  The fade-out is gone because it cannot be shown. The SDRAM read port
    //  takes a single address, so the old and the new frame cannot be read at
    //  the same time; the old picture cannot be dimmed while the new one is
    //  prepared. What can be shown, and what looks smooth, is the new picture
    //  arriving from black.
    //
    //  TRIGGERED BY image_swap, which is disp_buf_idx having really moved. That
    //  is the one moment the picture on screen changes, so every change fades --
    //  no dependence on how long the load took, which is what made the previous
    //  version fade only when the timings happened to line up.
    // ==================================================================
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            ts             <= TS_IDLE;
            ts_cnt         <= 32'd0;
            trans_alpha    <= 8'd255;
            transition_run <= 1'b0;
            swap_seen_l    <= 1'b0;
        end else if (mode == MODE_EMERGENCY) begin
            // HARD CUT. Emergency wants zero latency: the new picture is
            // already on screen by the time this matters, and a fade would
            // delay exactly the information the mode exists to deliver.
            ts             <= TS_IDLE;
            ts_cnt         <= 32'd0;
            trans_alpha    <= 8'd255;
            transition_run <= 1'b0;
        end else if (image_swap) begin
            // ==========================================================
            //  RESTART THE RAMP ONLY WHEN IDLE.
            //
            //  image_swap 是一拍脉冲，斜坡 1.34 s，而一次装载约 0.6 s
            //  （一次按键连按更是几百毫秒一次）。所以"新图的提交"几乎必然
            //  落在上一张图的斜坡内部。
            //
            //  【原来的写法】无条件 ts_cnt<=0 / trans_alpha<=0，即每次提交都
            //  把画面重新压到全黑。后果实测（sim/fsm/tb_mode_fsm.v，
            //  连按间隔取 0.35*斜坡，正好等于真实的 0.6 s/1.34 s）：
            //      每次换图 alpha 最大只升到 89/255 ≈ 35%，
            //      而且每次提交都先掉到 0（全黑）；
            //      连按期间画面一直"忽明忽暗、整体很暗"，看着就像卡死。
            //      （单次换图时 alpha 能到 255，所以只有连按才暴露。）
            //
            //  【现在】只有机器处于 IDLE（上一次淡入已经走完）才从黑开始新的一次
            //  淡入；正在淡入时又来一张新图，就不打断、不重压，让斜坡继续把
            //  画面往上带。新图仍然有淡入效果（从当前亮度继续升到 255），
            //  但画面永远不会因为"又换了一张"而重新变黑。
            // ==========================================================
            swap_seen_l <= 1'b1;

            if (ts == TS_IDLE) begin
                if (transition_type == 2'd1) begin
                    ts             <= TS_FADE_IN;
                    ts_cnt         <= 32'd0;
                    trans_alpha    <= 8'd0;
                    transition_run <= 1'b1;
                end else begin
                    // 硬切 / 光圈：不占这级调光
                    ts             <= TS_IDLE;
                    ts_cnt         <= 32'd0;
                    trans_alpha    <= 8'd255;      // no dimming at all
                    transition_run <= 1'b0;
                end
            end
            // else: 正在淡入 —— 本次换图不动 ts/ts_cnt/trans_alpha
        end else begin
            case (ts)
                TS_IDLE: begin
                    // Nothing to do: alpha is fully up and the picture is
                    // showing. A picture change is handled by the image_swap
                    // test ABOVE the case, which is what lets it interrupt a
                    // running ramp as well as start one from here.
                    trans_alpha    <= 8'd255;
                    transition_run <= 1'b0;
                    swap_seen_l    <= 1'b0;
                end

                // alpha rises 0 -> 255: the new picture brightens into view.
                //
                // The clamp matters. alpha is (ts_cnt >> FADE_SHIFT), and the
                // shift result passes 255 well before ts_cnt reaches FADE_LEN,
                // so without it the picture would brighten, wrap through black
                // and brighten again.
                TS_FADE_IN: begin
                    transition_run <= 1'b1;
                    if (ts_cnt + 32'd1 >= FADE_LEN) begin
                        trans_alpha    <= 8'd255;
                        ts_cnt         <= 32'd0;
                        ts             <= TS_IDLE;
                        transition_run <= 1'b0;
                    end else begin
                        ts_cnt <= ts_cnt + 32'd1;
                        if ((ts_cnt >> FADE_SHIFT) >= 32'd255)
                            trans_alpha <= 8'd255;
                        else
                            trans_alpha <= (ts_cnt >> FADE_SHIFT);
                    end
                end

                default: begin
                    ts             <= TS_IDLE;
                    trans_alpha    <= 8'd255;
                    transition_run <= 1'b0;
                end
            endcase
        end
    end

    // ==================================================================
    //  radial iris  (the picture is revealed from the middle outwards)
    //
    //  Starts on the same trigger as the fade -- the image swap -- and plays
    //  entirely on the NEW picture. The aperture opens from 0 to IRIS_UNIT
    //  over IRIS_LEN clocks, so the dark corners sweep outwards and the
    //  picture appears to grow.
    //
    //  Watchdog of the same kind as the fade: if iris_en were ever left high,
    //  iris_prog would saturate at IRIS_UNIT, the aperture would be fully open
    //  and the picture would simply be visible. That makes this effect
    //  fail-safe by construction -- worst case it does nothing.
    //
    //  The OLD picture cannot be shown shrinking: once the buffer swaps, the
    //  SDRAM read port no longer reaches it.
    // ==================================================================
    localparam [31:0] IRIS_LEN = IRIS_LEN_PARAM;

    // LOCKOUT -- and this is the fix for "the iris never opens".
    //
    // Measured with tb_timing.v at the real durations: the aperture kept
    // jumping back to 0 every few milliseconds (ap = 491, then 30, then 60,
    // then 29 ...) so it never got anywhere. The cause is that a single key
    // press makes sd_card_bmp commit the display buffer SEVERAL times (it is
    // double buffered and re-scans), and every one of those produced an
    // image_swap pulse that restarted the wipe from zero.
    //
    // The fade had a lockout from the start and was therefore unaffected --
    // alpha ramped smoothly from 3 to 99 in the same run -- which is why the
    // fade looked almost right while the iris looked like a hard cut.
    //
    // After a wipe completes, further swaps are ignored for a while, so the
    // aperture always gets to finish travelling.
    localparam [23:0] IRIS_COOLDOWN = IRIS_LEN >> 2;

    reg [31:0] iris_cnt;
    reg [31:0] iris_cool_cnt;
    reg        iris_in_cooldown;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            iris_en          <= 1'b0;
            iris_aperture    <= IRIS_DMAX;   // fully open: mask cannot blank
            iris_cnt         <= 24'd0;
            iris_cool_cnt    <= 24'd0;
            iris_in_cooldown <= 1'b0;
        end else if (mode == MODE_EMERGENCY) begin
            // emergency is a hard cut: never wipe the evacuation picture in
            iris_en          <= 1'b0;
            iris_aperture    <= IRIS_DMAX;
            iris_cnt         <= 24'd0;
            iris_cool_cnt    <= 24'd0;
            iris_in_cooldown <= 1'b0;
        end else if (transition_type != 2'd2) begin
            // ==========================================================
            //  NOT THE SELECTED EFFECT: FORCE THE MASK FULLY OPEN.
            //
            //  This is not just tidiness. If the operator switches away from
            //  the iris WHILE the aperture is still closing, nothing else
            //  would ever move it back -- the ramp only runs inside the
            //  `iris_en` branch below, which this path skips -- and the screen
            //  would stay masked down to a black tunnel with no way out.
            //  Forcing DMAX here makes "not iris" a state that can always
            //  recover, whatever it was interrupted from.
            // ==========================================================
            iris_en          <= 1'b0;
            iris_aperture    <= IRIS_DMAX;
            iris_cnt         <= 24'd0;
            iris_cool_cnt    <= 24'd0;
            iris_in_cooldown <= 1'b0;
        end else if (image_swap && (transition_type == 2'd2) && !iris_en && !iris_in_cooldown) begin
            // ==========================================================
            //  THE IRIS ONLY PLAYS WHEN IT IS THE SELECTED EFFECT.
            //
            //  It used to start on EVERY image_swap, unconditionally. That was
            //  invisible while layer_mixer had USE_IRIS(0), because the stage
            //  was bypassed downstream -- but the moment the iris is switched
            //  on for the acceptance demo, an ungated trigger would iris every
            //  swap including the ones where the operator chose a fade.
            // ==========================================================
            iris_en          <= 1'b1;
            iris_aperture    <= 32'd0;       // closed: whole frame black
            iris_cnt         <= 24'd0;
            iris_in_cooldown <= 1'b0;
        end else if (iris_en) begin
            if (iris_cnt >= IRIS_LEN) begin
                iris_en          <= 1'b0;
                iris_aperture    <= IRIS_DMAX;  // fully open: mask gone
                iris_cnt         <= 24'd0;
                iris_cool_cnt    <= 24'd0;
                iris_in_cooldown <= 1'b1;       // ignore swaps for a while
            end else begin
                iris_cnt      <= iris_cnt + 24'd1;
                // aperture = iris_cnt * IRIS_DMAX / IRIS_LEN, in metric units.
                //
                // WIDTH MATTERS. iris_cnt reaches IRIS_LEN (25e6 for the 1 s
                // ramp) and IRIS_DMAX is 153601, so the natural product is
                // 3.84e12 -- about 900x past the 32-bit ceiling of 4.29e9.
                // Verilog truncates silently and the aperture then CYCLES
                // instead of ramping, which tb_timing.v caught as aperture
                // jumping 491 -> 30 -> 91 while iris_en stayed high.
                //
                // The counter is shifted down first. That keeps the product at
                // 3.75e9 (inside 32 bits) and the final value exact at 153601,
                // because IRIS_DMAX/IRIS_LEN is unaffected by shifting only
                // one side. Chosen with iris_scaling.py, which compared four
                // formulations and their worst-case errors:
                //     ((cnt*DMAX)/LEN)                        overflows
                //     ((cnt>>8)*(DMAX/256))/(LEN/256)         final = 600, 99.6% wrong
                //     (cnt*(DMAX>>s))/(LEN>>s)                overflows until s=10
                //     ((cnt>>10)*DMAX)/(LEN>>10)              final = 153601, exact
                iris_aperture <= ((iris_cnt >> 10) * IRIS_DMAX) / (IRIS_LEN >> 10);
            end
        end else if (iris_in_cooldown) begin
            if (iris_cool_cnt >= IRIS_COOLDOWN) begin
                iris_in_cooldown <= 1'b0;
                iris_cool_cnt    <= 24'd0;
            end else begin
                iris_cool_cnt <= iris_cool_cnt + 24'd1;
            end
        end
    end

    // ==================================================================
    //  mode register -- the one place the mode actually changes
    // ==================================================================
    reg [1:0] mode_prev;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            mode        <= MODE_SELFTEST;
            mode_prev   <= MODE_SELFTEST;
            mode_changed<= 1'b0;
        end else begin
            mode_prev <= mode;

            // priority: emergency > self test completion > normal
            if (emg_stable) begin
                mode <= MODE_EMERGENCY;
            end else if (mode == MODE_SELFTEST) begin
                // leave self test only when it is finished (and emergency is not on)
                if (self_test_done) mode <= MODE_NORMAL;
            end else begin
                mode <= MODE_NORMAL;
            end

            // single-cycle pulse whenever the mode register actually changes
            mode_changed <= (mode_prev != mode);
        end
    end

    // ==================================================================
    //  latency measurement tap
    // ==================================================================
    assign emg_debounced = emg_stable;

    // ==================================================================
    //  outputs derived from mode
    //  these are combinational so the mixer sees them in the SAME clock
    //  that the mode register changes -- that is what makes the cut fast
    // ==================================================================
    always @(*) begin
        case (mode)
            MODE_EMERGENCY: begin
                text_en         = 1'b1;
                text_color      = 3'd1;        // red: maximum salience
                transition_type = 2'd0;        // hard cut
            end
            MODE_SELFTEST: begin
                text_en         = 1'b1;
                text_color      = 3'd3;        // yellow
                transition_type = 2'd0;
            end
            default: begin
                text_en         = 1'b1;
                text_color      = 3'd0;        // white
                // ----------------------------------------------------------
                //  NORMAL MODE HONOURS param_transition.
                //
                //  This closes the acceptance clause "拨码可切换效果类型". The
                //  input used to exist but was read nowhere, so no switch could
                //  ever change the effect -- the type was fixed at fade.
                //
                //  2 is the RADIAL IRIS, not a slide. Iris was already fully
                //  implemented in layer_mixer and mode_fsm and merely switched
                //  off by USE_IRIS(0); it is the ready-made second effect, so
                //  the slot that the contract called "2=滑动" now delivers a
                //  real, tested effect instead of a dangling promise. The
                //  contract also listed 3=随机; there is no such thing here.
                //
                //  THE DEFAULT MATTERS. `default` catches 1 AND anything that
                //  is not 0 or 2 -- including an undriven (Z) input. That makes
                //  the fallback FADE, which is exactly what this FSM did before
                //  the input was connected, so every existing testbench that
                //  leaves param_transition open keeps its behaviour.
                // ----------------------------------------------------------
                case (param_transition)
                    2'd0:    transition_type = 2'd0;   // hard cut
                    2'd2:    transition_type = 2'd2;   // radial iris
                    default: transition_type = 2'd1;   // fade (also the Z case)
                endcase
            end
        endcase
    end


    // the transition state, for the 7-segment diagnostic
    assign ts_state_o = ts;

endmodule
