// ============================================================================
//  secure_boot_chk.sv
//  Aquila — Secure Boot Checker (image hash + pubkey anchor + ECDSA verify + anti-rollback)
// ----------------------------------------------------------------------------
//  High-level flow:
//    1) Latch manifest (magic, version, img_len, img_hash, pubkey, signature, algo).
//    2) Stream image bytes into SHA-256; compare digest to manifest.img_hash.
//    3) Hash manifest public key and compare to OTP root key hash (or dev key hash if allowed).
//    4) Run ECDSA-P256 verify of signature over image digest (VERIFY_REPEATS times).
//    5) Enforce anti-rollback (manifest.sec_ver >= otp_min_sec_ver). Optionally request bump.
//    6) On success: pass_o pulse + boot_ok_o high. On failure: fail_o pulse + event out.
//
//  Notes:
//    - All interfaces are synchronous to clk_i.
//    - The image is provided as a ready/valid stream (128b + keep + last).
//    - Manifest is provided "out-of-band" once via valid/ready handshake.
//    - OTP anchors are hard inputs (sampled at start).
//    - Replace the black-box crypto stubs with your actual IP.
//
//  © 2025 Aquila Project. MIT-style license.
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module secure_boot_chk #(
  // ---------------------- Geometry ----------------------
  parameter int unsigned ADDR_W          = 48,     // (reserved for future AXI master variant)
  parameter int unsigned LEN_W           = 32,
  parameter int unsigned HASH_W          = 256,    // SHA-256
  parameter int unsigned EC_PK_W         = 512,    // P-256 pubkey (x||y)
  parameter int unsigned SIG_W           = 512,    // ECDSA r||s
  parameter int unsigned SECVER_W        = 32,

  // ---------------------- Stream geometry ---------------
  parameter int unsigned IMG_DATA_W      = 128,
  localparam int unsigned IMG_BYTES      = IMG_DATA_W/8,

  // ---------------------- Behavior knobs ----------------
  parameter int unsigned VERIFY_REPEATS  = 2,      // run signature verify N times (>=1)
  parameter int unsigned WDOG_BITS       = 28,     // watchdog counter width
  parameter int unsigned WDOG_HASH_MAX   = 28'd50_000_000, // cycles cap for hashing
  parameter int unsigned WDOG_SIG_MAX    = 28'd50_000_000, // cycles cap for signature

  // ---------------------- IDs / events ------------------
  parameter logic [7:0]   EVT_SRC_ID     = 8'h42   // "B" Boot
)(
  input  logic                          clk_i,
  input  logic                          rstn_i,          // sync, active-low

  // ======================= Control ======================
  input  logic                          start_i,         // pulse: capture manifest & start
  output logic                          busy_o,          // high while operation in progress
  output logic                          pass_o,          // pulse: success
  output logic                          fail_o,          // pulse: failure
  output logic                          boot_ok_o,       // latched pass (until reset)

  // ======================= Manifest in ==================
  // Provide once, alongside start_i (or before); captured when 'man_valid_i && man_ready_o'
  input  logic                          man_valid_i,
  output logic                          man_ready_o,
  input  logic [31:0]                   man_magic_i,
  input  logic [SECVER_W-1:0]           man_sec_ver_i,
  input  logic [LEN_W-1:0]              man_img_len_i,   // bytes (informational; stream length should match)
  input  logic [HASH_W-1:0]             man_img_hash_i,  // expected SHA-256 of image
  input  logic [EC_PK_W-1:0]            man_pubkey_i,    // Qx||Qy (big-endian per manifest spec)
  input  logic [SIG_W-1:0]              man_sig_i,       // r||s (big-endian per manifest spec)
  input  logic [7:0]                    man_key_algo_i,  // 0 = ECDSA-P256-SHA256 (supported)

  // ======================= Image stream =================
  input  logic                          img_valid_i,
  output logic                          img_ready_o,
  input  logic [IMG_DATA_W-1:0]         img_data_i,
  input  logic [IMG_BYTES-1:0]          img_keep_i,      // per-byte valid
  input  logic                          img_last_i,

  // ======================= OTP anchors =================
  input  logic [HASH_W-1:0]             otp_root_pubkey_hash_i,  // SHA-256 hash of allowed pubkey
  input  logic [HASH_W-1:0]             otp_dev_pubkey_hash_i,   // optional dev key hash (0 to disable)
  input  logic                          otp_allow_dev_keys_i,    // allow dev key if set
  input  logic [SECVER_W-1:0]           otp_min_sec_ver_i,       // anti-rollback floor

  // ======================= Anti-rollback bump ===========
  output logic                          bump_req_o,      // request OTP bump to sec_ver
  output logic [SECVER_W-1:0]           bump_to_ver_o,   // desired min version
  input  logic                          bump_ack_i,      // OTP controller acked the request (optional)
  input  logic                          bump_done_i,     // OTP controller completed (optional)
  input  logic                          bump_ok_i,       // OTP controller success (optional)

  // ======================= Event out (to err_logger) ====
  output logic                          evt_valid_o,
  input  logic                          evt_ready_i,
  output logic [7:0]                    evt_src_o,
  output logic [1:0]                    evt_sev_o,       // 2=ERR, 3=FATAL
  output logic [11:0]                   evt_code_o,
  output logic [31:0]                   evt_info_o       // e.g., state-specific info
);

  // --------------------------------------------------------------------------
  // Constants / error codes / magic
  // --------------------------------------------------------------------------
  localparam logic [31:0] MAN_MAGIC = 32'h4151_5541; // "AQUA"

  typedef enum logic [11:0] {
    EC_OK                     = 12'h000,
    EC_BAD_MAGIC              = 12'h001,
    EC_UNSUPPORTED_ALGO       = 12'h002,
    EC_HASH_MISMATCH          = 12'h003,
    EC_PUBKEY_HASH_MISMATCH   = 12'h004,
    EC_SIG_VERIFY_FAIL        = 12'h005,
    EC_ROLLBACK               = 12'h006,
    EC_HASH_TIMEOUT           = 12'h007,
    EC_SIG_TIMEOUT            = 12'h008,
    EC_INTERNAL               = 12'h0FF
  } err_code_e;

  // --------------------------------------------------------------------------
  // State
  // --------------------------------------------------------------------------
  typedef enum logic [3:0] {
    S_IDLE,
    S_LATCH,
    S_HASH_IMG,
    S_HASH_IMG_WAIT,
    S_COMPARE_IMG_HASH,
    S_HASH_PUBKEY_LOAD,
    S_HASH_PUBKEY_FEED,
    S_HASH_PUBKEY_WAIT,
    S_COMPARE_PUBKEY_HASH,
    S_SIG_START,
    S_SIG_WAIT,
    S_SIG_REPEAT_CHECK,
    S_ANTI_ROLLBACK,
    S_BUMP_REQ,
    S_PASS,
    S_FAIL
  } state_e;

  state_e state_q, state_d;

  // Latches for manifest
  logic [31:0]          man_magic_q;
  logic [SECVER_W-1:0]  man_sec_ver_q;
  logic [LEN_W-1:0]     man_img_len_q;
  logic [HASH_W-1:0]    man_img_hash_q;
  logic [EC_PK_W-1:0]   man_pubkey_q;
  logic [SIG_W-1:0]     man_sig_q;
  logic [7:0]           man_algo_q;

  // Sticky anchors (sampled at start)
  logic [HASH_W-1:0]    otp_root_pubkey_hash_q, otp_dev_pubkey_hash_q;
  logic                 otp_allow_dev_keys_q;
  logic [SECVER_W-1:0]  otp_min_sec_ver_q;

  // Control flags / results
  logic [HASH_W-1:0]    img_digest_q;      // computed SHA256(image)
  logic [HASH_W-1:0]    pk_digest_q;       // computed SHA256(pubkey)
  logic [HASH_W-1:0]    pk_digest_sel_w;   // which OTP hash to compare to

  logic [11:0]          last_error_q, last_error_d;

  logic [WDOG_BITS-1:0] wdog_q, wdog_d;
  logic [7:0]           sig_repeat_q, sig_repeat_d;

  // Outputs
  logic busy_d, pass_d, fail_d, boot_ok_d;

  // Event
  logic evt_v_q, evt_v_d;
  logic [1:0] evt_sev_q, evt_sev_d;
  logic [11:0] evt_code_q, evt_code_d;
  logic [31:0] evt_info_q, evt_info_d;

  // Zeroization request (after PASS/FAIL)
  logic zeroize_w;

  // --------------------------------------------------------------------------
  // Manifest capture handshake
  // --------------------------------------------------------------------------
  assign man_ready_o = (state_q == S_IDLE);

  // --------------------------------------------------------------------------
  // SHA-256 stream for image hashing (pass-through from input stream)
  // --------------------------------------------------------------------------
  // The SHA engine consumes data only in S_HASH_IMG.* states.
  logic sha_img_in_valid, sha_img_in_ready, sha_img_in_last;
  logic [IMG_DATA_W-1:0] sha_img_in_data;
  logic [IMG_BYTES-1:0]  sha_img_in_keep;

  logic sha_img_dgst_valid;
  logic [HASH_W-1:0] sha_img_dgst;

  // Drive image stream to SHA during HASH_IMG
  assign sha_img_in_valid = (state_q == S_HASH_IMG) ? img_valid_i : 1'b0;
  assign sha_img_in_last  = (state_q == S_HASH_IMG) ? img_last_i  : 1'b0;
  assign sha_img_in_data  = img_data_i;
  assign sha_img_in_keep  = img_keep_i;

  assign img_ready_o      = (state_q == S_HASH_IMG) ? sha_img_in_ready : 1'b0;

  sha256_stream #(
    .DATA_W    (IMG_DATA_W),
    .KEEP_W    (IMG_BYTES)
  ) u_sha_img (
    .clk_i     (clk_i),
    .rstn_i    (rstn_i),
    .start_i   (state_q == S_HASH_IMG && (sha_img_in_valid && sha_img_in_ready) && (wdog_q == '0)), // start implicit
    .data_valid_i (sha_img_in_valid),
    .data_ready_o (sha_img_in_ready),
    .data_keep_i  (sha_img_in_keep),
    .data_last_i  (sha_img_in_last),
    .data_i       (sha_img_in_data),
    .digest_valid_o(sha_img_dgst_valid),
    .digest_o     (sha_img_dgst)
  );

  // --------------------------------------------------------------------------
  // SHA-256 for pubkey hashing (one-shot, 64 bytes from manifest)
  // --------------------------------------------------------------------------
  logic sha_pk_in_valid, sha_pk_in_ready, sha_pk_in_last;
  logic [IMG_DATA_W-1:0] sha_pk_in_data;
  logic [IMG_BYTES-1:0]  sha_pk_in_keep;
  logic sha_pk_dgst_valid;
  logic [HASH_W-1:0] sha_pk_dgst;

  sha256_stream #(
    .DATA_W    (IMG_DATA_W),
    .KEEP_W    (IMG_BYTES)
  ) u_sha_pk (
    .clk_i     (clk_i),
    .rstn_i    (rstn_i),
    .start_i   (state_q == S_HASH_PUBKEY_FEED && sha_pk_in_valid && sha_pk_in_ready && (wdog_q == '0)),
    .data_valid_i (sha_pk_in_valid),
    .data_ready_o (sha_pk_in_ready),
    .data_keep_i  (sha_pk_in_keep),
    .data_last_i  (sha_pk_in_last),
    .data_i       (sha_pk_in_data),
    .digest_valid_o(sha_pk_dgst_valid),
    .digest_o     (sha_pk_dgst)
  );

  // Feed pubkey as 4 beats (512b) of 128b; little-endian chunking of man_pubkey_q
  // (Adjust if your manifest uses big-endian chunk order; hash must match OTP preimage.)
  logic [1:0] pk_beat_q, pk_beat_d; // 0..3
  always_comb begin
    sha_pk_in_valid = 1'b0;
    sha_pk_in_last  = 1'b0;
    sha_pk_in_keep  = {IMG_BYTES{1'b1}};
    sha_pk_in_data  = '0;

    if (state_q == S_HASH_PUBKEY_FEED) begin
      sha_pk_in_valid = 1'b1;
      unique case (pk_beat_q)
        2'd0: sha_pk_in_data = man_pubkey_q[127:0];
        2'd1: sha_pk_in_data = man_pubkey_q[255:128];
        2'd2: sha_pk_in_data = man_pubkey_q[383:256];
        2'd3: sha_pk_in_data = man_pubkey_q[511:384];
      endcase
      sha_pk_in_last = (pk_beat_q == 2'd3);
    end
  end

  // --------------------------------------------------------------------------
  // ECDSA-P256 verify (r||s) over SHA-256(image); Q = pubkey (x||y)
  // --------------------------------------------------------------------------
  logic ecdsa_start, ecdsa_busy, ecdsa_valid, ecdsa_pass;

  ecdsa_p256_verify u_ecdsa (
    .clk_i     (clk_i),
    .rstn_i    (rstn_i),
    .start_i   (ecdsa_start),
    .hash_i    (img_digest_q),               // 256b
    .qx_i      (man_pubkey_q[255:0]),        // adjust byte order to your core
    .qy_i      (man_pubkey_q[511:256]),
    .r_i       (man_sig_q[255:0]),
    .s_i       (man_sig_q[511:256]),
    .busy_o    (ecdsa_busy),
    .valid_o   (ecdsa_valid),
    .pass_o    (ecdsa_pass)
  );

  // --------------------------------------------------------------------------
  // Main FSM
  // --------------------------------------------------------------------------
  // Busy/pass/fail pulses
  assign busy_o = (state_q != S_IDLE) && (state_q != S_PASS) && (state_q != S_FAIL);

  // Selected OTP pubkey hash (prod vs dev)
  always_comb begin
    pk_digest_sel_w = otp_root_pubkey_hash_q;
    if (otp_allow_dev_keys_q && (otp_dev_pubkey_hash_q != '0))
      pk_digest_sel_w = otp_dev_pubkey_hash_q;
  end

  // Next-state defaults
  always_comb begin
    state_d       = state_q;
    last_error_d  = last_error_q;
    pass_d        = 1'b0;
    fail_d        = 1'b0;
    boot_ok_d     = boot_ok_o;

    // SHA PK beat ctr
    pk_beat_d     = pk_beat_q;

    // ECDSA
    ecdsa_start   = 1'b0;
    sig_repeat_d  = sig_repeat_q;

    // Watchdog
    wdog_d        = (state_q == S_IDLE || state_q == S_LATCH) ? '0 : (wdog_q + {{WDOG_BITS-1{1'b0}},1'b1});

    // Bump request (level for 1 cycle in S_BUMP_REQ)
    bump_req_o    = 1'b0;
    bump_to_ver_o = man_sec_ver_q;

    case (state_q)
      S_IDLE: begin
        if (start_i && man_valid_i) begin
          // Latch in sequential block
          state_d      = S_LATCH;
        end
      end

      S_LATCH: begin
        // Sanity checks
        if (man_magic_q != MAN_MAGIC) begin
          last_error_d = EC_BAD_MAGIC;
          state_d      = S_FAIL;
        end else if (man_algo_q != 8'd0) begin
          last_error_d = EC_UNSUPPORTED_ALGO;
          state_d      = S_FAIL;
        end else begin
          // Start hashing image
          state_d      = S_HASH_IMG;
        end
      end

      S_HASH_IMG: begin
        // Wait for SHA digest; the stream drives the SHA directly
        if (sha_img_dgst_valid) begin
          state_d = S_COMPARE_IMG_HASH;
        end else if (wdog_q >= WDOG_HASH_MAX[WDOG_BITS-1:0]) begin
          last_error_d = EC_HASH_TIMEOUT;
          state_d      = S_FAIL;
        end
      end

      S_COMPARE_IMG_HASH: begin
        if (sha_img_dgst == man_img_hash_q) begin
          state_d = S_HASH_PUBKEY_LOAD;
        end else begin
          last_error_d = EC_HASH_MISMATCH;
          state_d      = S_FAIL;
        end
      end

      S_HASH_PUBKEY_LOAD: begin
        pk_beat_d = 2'd0;
        state_d   = S_HASH_PUBKEY_FEED;
      end

      S_HASH_PUBKEY_FEED: begin
        if (sha_pk_in_valid && sha_pk_in_ready) begin
          if (pk_beat_q == 2'd3) begin
            state_d = S_HASH_PUBKEY_WAIT;
          end else begin
            pk_beat_d = pk_beat_q + 2'd1;
          end
        end
      end

      S_HASH_PUBKEY_WAIT: begin
        if (sha_pk_dgst_valid) begin
          state_d = S_COMPARE_PUBKEY_HASH;
        end else if (wdog_q >= WDOG_HASH_MAX[WDOG_BITS-1:0]) begin
          last_error_d = EC_HASH_TIMEOUT;
          state_d      = S_FAIL;
        end
      end

      S_COMPARE_PUBKEY_HASH: begin
        if (pk_digest_q == pk_digest_sel_w) begin
          // Run ECDSA verify (potentially multiple times)
          sig_repeat_d = (VERIFY_REPEATS == 0) ? 8'd1 : VERIFY_REPEATS[7:0];
          state_d      = S_SIG_START;
        } else begin
          last_error_d = EC_PUBKEY_HASH_MISMATCH;
          state_d      = S_FAIL;
        end
      end

      S_SIG_START: begin
        ecdsa_start = 1'b1;
        state_d     = S_SIG_WAIT;
        wdog_d      = '0;
      end

      S_SIG_WAIT: begin
        if (ecdsa_valid) begin
          if (ecdsa_pass) begin
            state_d = S_SIG_REPEAT_CHECK;
          end else begin
            last_error_d = EC_SIG_VERIFY_FAIL;
            state_d      = S_FAIL;
          end
        end else if (wdog_q >= WDOG_SIG_MAX[WDOG_BITS-1:0]) begin
          last_error_d = EC_SIG_TIMEOUT;
          state_d      = S_FAIL;
        end
      end

      S_SIG_REPEAT_CHECK: begin
        if (sig_repeat_q > 8'd1) begin
          sig_repeat_d = sig_repeat_q - 8'd1;
          state_d      = S_SIG_START;
        end else begin
          state_d      = S_ANTI_ROLLBACK;
        end
      end

      S_ANTI_ROLLBACK: begin
        if (man_sec_ver_q < otp_min_sec_ver_q) begin
          last_error_d = EC_ROLLBACK;
          state_d      = S_FAIL;
        end else if (man_sec_ver_q > otp_min_sec_ver_q) begin
          state_d      = S_BUMP_REQ;
        end else begin
          state_d      = S_PASS;
        end
      end

      S_BUMP_REQ: begin
        bump_req_o    = 1'b1;
        bump_to_ver_o = man_sec_ver_q;
        // We can optionally wait for bump_done_i/bump_ok_i; non-blocking here.
        state_d       = S_PASS;
      end

      S_PASS: begin
        pass_d     = 1'b1;
        boot_ok_d  = 1'b1;
        state_d    = S_IDLE;
      end

      S_FAIL: begin
        fail_d     = 1'b1;
        boot_ok_d  = 1'b0;
        state_d    = S_IDLE;
      end

      default: state_d = S_IDLE;
    endcase
  end

  // --------------------------------------------------------------------------
  // Sequential section: latches, results, watchdog, zeroization
  // --------------------------------------------------------------------------
  // Sensitive regs to zeroize after terminal states
  assign zeroize_w = (state_q == S_PASS) || (state_q == S_FAIL);

  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      state_q         <= S_IDLE;

      man_magic_q     <= 32'h0;
      man_sec_ver_q   <= '0;
      man_img_len_q   <= '0;
      man_img_hash_q  <= '0;
      man_pubkey_q    <= '0;
      man_sig_q       <= '0;
      man_algo_q      <= 8'h0;

      otp_root_pubkey_hash_q <= '0;
      otp_dev_pubkey_hash_q  <= '0;
      otp_allow_dev_keys_q   <= 1'b0;
      otp_min_sec_ver_q      <= '0;

      img_digest_q     <= '0;
      pk_digest_q      <= '0;

      last_error_q     <= EC_OK;
      wdog_q           <= '0;
      pk_beat_q        <= 2'd0;
      sig_repeat_q     <= 8'd0;

      pass_o           <= 1'b0;
      fail_o           <= 1'b0;
      boot_ok_o        <= 1'b0;

    end else begin
      state_q <= state_d;

      // Pulses
      pass_o  <= pass_d;
      fail_o  <= fail_d;
      if (pass_d) boot_ok_o <= 1'b1;
      if (fail_d) boot_ok_o <= 1'b0;

      // Manifest capture at start
      if (state_q == S_IDLE && start_i && man_valid_i) begin
        man_magic_q     <= man_magic_i;
        man_sec_ver_q   <= man_sec_ver_i;
        man_img_len_q   <= man_img_len_i;
        man_img_hash_q  <= man_img_hash_i;
        man_pubkey_q    <= man_pubkey_i;
        man_sig_q       <= man_sig_i;
        man_algo_q      <= man_key_algo_i;

        otp_root_pubkey_hash_q <= otp_root_pubkey_hash_i;
        otp_dev_pubkey_hash_q  <= otp_dev_pubkey_hash_i;
        otp_allow_dev_keys_q   <= otp_allow_dev_keys_i;
        otp_min_sec_ver_q      <= otp_min_sec_ver_i;

        last_error_q     <= EC_OK;
      end

      // Hash digests latch when valid
      if (sha_img_dgst_valid) img_digest_q <= sha_img_dgst;
      if (sha_pk_dgst_valid)  pk_digest_q  <= sha_pk_dgst;

      // Watchdog, beats, repeats
      wdog_q       <= wdog_d;
      pk_beat_q    <= pk_beat_d;
      sig_repeat_q <= sig_repeat_d;

      // Error code
      last_error_q <= last_error_d;

      // Zeroize sensitive on terminal states
      if (zeroize_w) begin
        man_pubkey_q <= '0;
        man_sig_q    <= '0;
      end
    end
  end

  // --------------------------------------------------------------------------
  // Event reporting (to err_logger)
  // --------------------------------------------------------------------------
  // On failure, raise a single event with severity based on error.
  //  - FATAL for BAD_MAGIC, UNSUPPORTED_ALGO, SIG_TIMEOUT
  //  - ERR   for others
  function automatic logic [1:0] sev_from_err(input err_code_e e);
    case (e)
      EC_BAD_MAGIC, EC_UNSUPPORTED_ALGO, EC_SIG_TIMEOUT: return 2'd3; // FATAL
      default: return 2'd2; // ERR
    endcase
  endfunction

  always_comb begin
    evt_v_d    = evt_v_q;
    evt_sev_d  = evt_sev_q;
    evt_code_d = evt_code_q;
    evt_info_d = evt_info_q;

    // Fire on transition to S_FAIL
    if (fail_d) begin
      evt_v_d    = 1'b1;
      evt_sev_d  = sev_from_err(last_error_d);
      evt_code_d = last_error_d;
      // Provide some context in info (lower 16b state, upper 16b wdog)
      evt_info_d = { {16{1'b0}}, 8'(state_q), 8'(sig_repeat_q) };
    end else if (evt_v_q && evt_ready_i) begin
      // Handshake complete, drop
      evt_v_d = 1'b0;
    end
  end

  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      evt_v_q    <= 1'b0;
      evt_sev_q  <= 2'd0;
      evt_code_q <= 12'd0;
      evt_info_q <= 32'd0;
    end else begin
      evt_v_q    <= evt_v_d;
      evt_sev_q  <= evt_sev_d;
      evt_code_q <= evt_code_d;
      evt_info_q <= evt_info_d;
    end
  end

  assign evt_valid_o = evt_v_q;
  assign evt_sev_o   = evt_sev_q;
  assign evt_code_o  = evt_code_q;
  assign evt_info_o  = evt_info_q;
  assign evt_src_o   = EVT_SRC_ID;

  // --------------------------------------------------------------------------
  // Assertions
  // --------------------------------------------------------------------------
`ifdef ASSERT_ON
  // Start must coincide with manifest valid in IDLE
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    start_i |-> man_valid_i && man_ready_o)
    else $error("secure_boot_chk: start without manifest.");

  // Image stream only consumed in HASH_IMG
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    img_valid_i && !img_ready_o |-> (state_q != S_HASH_IMG))
    else $warning("secure_boot_chk: img_valid held while not hashing.");

  // Verify repeats >=1
  initial begin
    if (VERIFY_REPEATS < 1) $error("secure_boot_chk: VERIFY_REPEATS must be >= 1.");
  end
`endif

endmodule

// ============================================================================
//  Black-box crypto stubs (replace with real IP). Guarded with macros.
// ============================================================================

`ifndef AQUILA_HAVE_SHA256_STREAM
(* black_box *)
module sha256_stream #(
  parameter int unsigned DATA_W = 128,
  parameter int unsigned KEEP_W = DATA_W/8
)(
  input  logic                   clk_i,
  input  logic                   rstn_i,
  input  logic                   start_i,        // optional
  input  logic                   data_valid_i,
  output logic                   data_ready_o,
  input  logic [KEEP_W-1:0]      data_keep_i,
  input  logic                   data_last_i,
  input  logic [DATA_W-1:0]      data_i,
  output logic                   digest_valid_o,
  output logic [255:0]           digest_o
);
  // Synthesis stub only; replace in integration.
  // To avoid creating timing paths accidentally, tie-offs below keep it quiescent.
  assign data_ready_o    = 1'b1;
  assign digest_valid_o  = 1'b0;
  assign digest_o        = '0;
endmodule
`endif

`ifndef AQUILA_HAVE_ECDSA_P256_VERIFY
(* black_box *)
module ecdsa_p256_verify (
  input  logic         clk_i,
  input  logic         rstn_i,
  input  logic         start_i,
  input  logic [255:0] hash_i,
  input  logic [255:0] qx_i,
  input  logic [255:0] qy_i,
  input  logic [255:0] r_i,
  input  logic [255:0] s_i,
  output logic         busy_o,
  output logic         valid_o,
  output logic         pass_o
);
  // Synthesis stub only; replace in integration.
  assign busy_o  = 1'b0;
  assign valid_o = 1'b0;
  assign pass_o  = 1'b0;
endmodule
`endif

`default_nettype wire
