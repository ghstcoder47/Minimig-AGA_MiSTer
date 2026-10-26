// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dawid Janas
//
// FPGA endpoint for Main_MiSTer's USB capture stream, entirely in clk_sys.
// hps_ext.v supplies 16-bit words (left, then right) from window 0xF600.
// A FIFO separates Linux bursts from the 48 kHz consumer. Minimig uses out_u8
// for CIA-A PRB while active; out_l/out_r retain the transport words.
// The Amiga reads a free-running sample value, without a CIA FLAG handshake.
//
// This source file is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

module usb_sampler #(parameter CLK_RATE = 28375160, parameter CLK_RATE_NTSC = 28636360, parameter IDLE_TICKS = 24000)
(
	input             clk,
	input             reset,
	input             signed_out, // 1: signed 8 bit output (silence 0x00), 0: offset binary (0x80)
	input       [1:0] dither_mode, // 0 off (rounding), 1 RPDF +-0.5 LSB, 2 TPDF +-1 LSB, 3 TPDF gated by silence
	input             ntsc,       // selects the clk_sys frequency for the 48 kHz divider

	// hps_ext.v uses the same clock; start precedes the first payload word.
	input             start,      // pulse: new write transaction, resets word phase
	input             wr,         // pulse: one 16-bit word in din
	input      [15:0] din,
	output            req,        // status bit 14: room for a 512-frame burst

	// Minimig switches PRB back to the joystick inputs when active expires.
	output            active,     // recent ARM transaction, not a FIFO-nonempty flag
	output reg        ce,         // pulse at 48 kHz when out_l/out_r updated
	output reg [15:0] out_l,
	output reg [15:0] out_r,
	output      [7:0] out_u8      // PRB byte; Linux may pre-bias words for signed sampling
);

localparam AW = 11;               // 2048 frames, ~42 ms

reg  [31:0] mem[2**AW];
// The extra pointer bit distinguishes empty from full after address wrap.
reg  [AW:0] wr_ptr, rd_ptr;
reg  [15:0] left;
reg         phase;

// Request below half-full leaves room for the host's complete 512-frame burst.
wire [AW:0] level = wr_ptr - rd_ptr;
assign req = (level < 2**(AW-1));

always @(posedge clk) begin
	reg [31:0] q;

	if (reset) begin
		wr_ptr <= 0;
		phase  <= 0;
	end
	else begin
		// Discard an unmatched left word when a new transaction begins.
		if (start) phase <= 0;

		if (wr) begin
			phase <= ~phase;
			if (!phase) left <= din;
			else if (level != 2**AW) begin
				mem[wr_ptr[AW-1:0]] <= {left, din};
				wr_ptr <= wr_ptr + 1'd1;
			end
		end
	end
end

// RAM reads take one clock. On empty-to-nonempty transitions the pointers can
// report data before rd_q contains it; rd_q_ok prevents consuming that stale word.
reg [31:0] rd_q;
reg        rd_q_ok;
always @(posedge clk) begin
	rd_q    <= mem[rd_ptr[AW-1:0]];
	rd_q_ok <= (level != 0);
end

// clk_sys changes between PAL and NTSC. A fractional accumulator preserves
// 48 kHz on average instead of rounding the number of clocks per sample.
reg         cen_48k;
reg  [31:0] cen_cnt;
wire [31:0] clk_rate    = ntsc ? CLK_RATE_NTSC : CLK_RATE;
wire [31:0] cen_cnt_next = cen_cnt + 32'd48000;

always @(posedge clk) begin
	cen_48k <= 0;
	cen_cnt <= cen_cnt_next;
	if (reset) cen_cnt <= 0;
	else if (cen_cnt_next >= clk_rate) begin
		cen_48k <= 1;
		cen_cnt <= cen_cnt_next - clk_rate;
	end
end

reg [15:0] idle_cnt;
assign active = |idle_cnt;

always @(posedge clk) begin
	ce <= 0;

	if (reset) begin
		rd_ptr   <= 0;
		idle_cnt <= 0;
		out_l    <= 0;
		out_r    <= 0;
	end
	else begin
		if (start) idle_cnt <= IDLE_TICKS; // default timeout: 0.5 s at 48 kHz

		if (cen_48k) begin
			if (idle_cnt && !start) idle_cnt <= idle_cnt - 1'd1;
			// An underrun holds the last output; the inactivity timeout is separate.
			if (level != 0 && rd_q_ok) begin
				out_l  <= rd_q[31:16];
				out_r  <= rd_q[15:0];
				rd_ptr <= rd_ptr + 1'd1;
				ce     <= 1;
			end
		end
	end
end

// Difference of two 9-bit pseudo-random values: -511..511 in the L+R domain.
// One 8-bit output step is 512 here. The triangular dither is intended to
// decorrelate quantization error from quiet input signals.
reg [31:0] lfsr_a = 32'h1badcafe, lfsr_b = 32'h2fd3a5c1;
always @(posedge clk) begin
	lfsr_a <= {lfsr_a[30:0], lfsr_a[31] ^ lfsr_a[21] ^ lfsr_a[1] ^ lfsr_a[0]};
	lfsr_b <= {lfsr_b[30:0], lfsr_b[31] ^ lfsr_b[21] ^ lfsr_b[1] ^ lfsr_b[0]};
end

// Silence gate for mode 3: after 50 ms below one 8-bit step the dither stops,
// so an idle input stays quiet. Any louder sample reopens it immediately.
wire signed [16:0] mix  = $signed(out_l) + $signed(out_r);
wire        quiet = (mix > -17'sd512) && (mix < 17'sd512);
reg  [11:0] quiet_cnt = 0;
wire        gate_closed = (quiet_cnt >= 12'd2400);

// Modes 0, 1 and a closed gate use a plain floor, so a constant of 256
// (half an 8-bit step) provides rounding; RPDF adds 0..511 (a full step).
reg signed [9:0] dither;
always @(posedge clk) if (ce) begin
	quiet_cnt <= !quiet ? 12'd0 : (gate_closed ? quiet_cnt : quiet_cnt + 1'd1);
	case (dither_mode)
		2'd0: dither <= 10'sd256;
		2'd1: dither <= $signed({1'b0, lfsr_a[8:0]});
		2'd2: dither <= $signed({1'b0, lfsr_a[8:0]}) - $signed({1'b0, lfsr_b[8:0]});
		2'd3: dither <= gate_closed ? 10'sd256 : ($signed({1'b0, lfsr_a[8:0]}) - $signed({1'b0, lfsr_b[8:0]}));
	endcase
end

// The 18-bit sum retains dither overflow for saturation before reduction.
// Words arrive as true signed 16 bit samples. Dither and saturation work in
// that signed domain; only the final byte is re-encoded for the selected mode.
wire signed [17:0] mixd = mix + dither;
wire        sat    = (mixd[17] != mixd[16]);
wire [7:0]  s8     = sat ? (mixd[17] ? 8'h80 : 8'h7F) : mixd[16:9];
assign out_u8 = signed_out ? s8 : (s8 ^ 8'h80);

endmodule
