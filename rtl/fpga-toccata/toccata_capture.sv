
/********************************************/
/* toccata_record.v                         */
/* Toccata sound record                     */
/*                                          */
/* 2022-2023, ranzbak@gmail.com             */
/* USB audio source, 2026, Dawid Janas      */
/********************************************/

// The record FIFO is fed from the 48 kHz stream of usb_sampler. Samples are
// averaged (boxcar) down to the AD1848 sample rate selected by the driver and
// packed like the real codec: 8 bit unsigned, or 16 bit little endian two's
// complement; stereo is left then right.


module toccata_capture #(
    parameter int CLK_FREQUENCY = 28_359_380,
    parameter int FIFO_SIZE = 8192
) (
    input  logic        clk,
    input  logic        rst,

    // Control parameters
    input  wire         cen,        // Capture, disable - 0, enable - 1
    input  wire  [2:0]  freq_sel,   // See table below
    input  wire         sm,         // 0 - Mono, 1 - Stereo
    input  wire         fmt,        // 0 - 8 bit unsigned, 1 - 16 bit 2 complement
    input  wire         lc,         // 1 - companded, handled as 8 bit like playback
    input  wire         css,        // 0 - 24.576, 1 - 16.9344

    // Audio source: usb_sampler, 48 kHz strobe with signed 16 bit samples
    input  wire         src_ce,
    input  wire signed [15:0] src_l,
    input  wire signed [15:0] src_r,
    input  wire         src_active, // stream is running, otherwise record silence

    // FIFO interface
    output logic [7:0]  data_out,   // Head byte of the FIFO
    input  logic        rd,         // One clock pulse: head byte has been consumed
    output logic        empty,      // 1 - FIFO is empty
    output logic        half_full,  // 1 - FIFO is half full
    output logic        full,       // 1 - FIFO is full

    // Audio interface
    output logic [12:0] dbg,        // {hold[4:0], overrun[3:0], read-empty[3:0]} saturating, cleared on flush
    output logic        endata      // Strobe on new sample data
);

// FIFO dimensions
localparam int FIFO_FULL  = FIFO_SIZE;
localparam int FIFO_HALF  = 512; // Interrupt threshold of the library (reads 512 bytes per interrupt)
localparam int FIFO_EMPTY = 0;


// Frequency and frequency deviders
// AD1848 = 24.576MHz
// Deviders :
// 3072 - 8kHz
// 1536 - 16kHz
// 896  - 27.43kHz
// 768  - 31.27kHz
// 448  - 54.86kHz // Net supported in hardware
// 384  - 64kHz    // Net supported in hardware
// 512  - 48kHz
// 2560 - 9.6kHz
// Deviders based on CLK_FREQUENCY 24.576MHz
localparam int DEV_0_8_KHZ = CLK_FREQUENCY / 8000;
localparam int DEV_0_16_KHZ = CLK_FREQUENCY / 16000;
localparam int DEV_0_27_43_KHZ = CLK_FREQUENCY / 27430;
localparam int DEV_0_31_27_KHZ = CLK_FREQUENCY / 31270;
localparam int DEV_0_54_86_KHZ = CLK_FREQUENCY / 54860;
localparam int DEV_0_64_KHZ = CLK_FREQUENCY / 64000;
localparam int DEV_0_48_KHZ = CLK_FREQUENCY / 48000;
localparam int DEV_0_9_6_KHZ = CLK_FREQUENCY / 9600;
// Deviders based on CLK_FREQUENCY 16.9344MHz
localparam int DEV_1_5_5125_KHZ = CLK_FREQUENCY / 5512;
localparam int DEV_1_11_025_KHZ = CLK_FREQUENCY / 11025;
localparam int DEV_1_18_9_KHZ = CLK_FREQUENCY / 18900;
localparam int DEV_1_22_05_KHZ = CLK_FREQUENCY / 22050;
localparam int DEV_1_37_8_KHZ = CLK_FREQUENCY / 37800;
localparam int DEV_1_44_1_KHZ = CLK_FREQUENCY / 44100;
localparam int DEV_1_33_075_KHZ = CLK_FREQUENCY / 33075;
localparam int DEV_1_6_615_KHZ = CLK_FREQUENCY / 6615;

// Frequency delay counter number of bits needed
// to contain the devider for the audio frequencies
localparam int DELAY_COUNTER_BITS = $clog2(DEV_1_5_5125_KHZ) - 1;

// Audio devider reg
logic [DELAY_COUNTER_BITS:0] audio_dev = DEV_1_5_5125_KHZ[DELAY_COUNTER_BITS:0];
logic [DELAY_COUNTER_BITS:0] audio_delay = 0;
always_ff @(posedge clk) begin
    // Sample frequency devider selection, by xtal and register
    case ({css, freq_sel})
        // CSS is 0, 24.576 MHz
        4'h0:
            audio_dev <= DEV_0_8_KHZ[DELAY_COUNTER_BITS:0];
        4'h1:
            audio_dev <= DEV_0_16_KHZ[DELAY_COUNTER_BITS:0];
        4'h2:
            audio_dev <= DEV_0_27_43_KHZ[DELAY_COUNTER_BITS:0];
        4'h3:
            audio_dev <= DEV_0_31_27_KHZ[DELAY_COUNTER_BITS:0];
        4'h4:
            audio_dev <= DEV_0_54_86_KHZ[DELAY_COUNTER_BITS:0]; // Not supported in real hardware
        4'h5:
            audio_dev <= DEV_0_64_KHZ[DELAY_COUNTER_BITS:0]; // Not supported in real hardwark
        4'h6:
            audio_dev <= DEV_0_48_KHZ[DELAY_COUNTER_BITS:0];
        4'h7:
            audio_dev <= DEV_0_9_6_KHZ[DELAY_COUNTER_BITS:0];
        // CSS is 1, 16.9344 MHz
        4'h8:
            audio_dev <= DEV_1_5_5125_KHZ[DELAY_COUNTER_BITS:0];
        4'h9:
            audio_dev <= DEV_1_11_025_KHZ[DELAY_COUNTER_BITS:0];
        4'ha:
            audio_dev <= DEV_1_18_9_KHZ[DELAY_COUNTER_BITS:0];
        4'hb:
            audio_dev <= DEV_1_22_05_KHZ[DELAY_COUNTER_BITS:0];
        4'hc:
            audio_dev <= DEV_1_37_8_KHZ[DELAY_COUNTER_BITS:0];
        4'hd:
            audio_dev <= DEV_1_44_1_KHZ[DELAY_COUNTER_BITS:0];
        4'he:
            audio_dev <= DEV_1_33_075_KHZ[DELAY_COUNTER_BITS:0];
        4'hf:
            audio_dev <= DEV_1_6_615_KHZ[DELAY_COUNTER_BITS:0];
    endcase

    `ifdef DEBUG
    // $display("audio_dev = %0d", audio_dev);
    // In order to not make the simulation too long,
    // We replace the delay with a value of 5 during simulations.
    audio_dev <= 20;
    `endif
end

localparam int AW = $clog2(FIFO_SIZE);
logic       ca_en;            // Sample rate strobe

// ---------------------------------------------------------------------------
// FIFO, first-word style: data_out always shows the head byte.
// ---------------------------------------------------------------------------
logic [7:0] mem [FIFO_SIZE];
logic [AW:0] wr_ptr, rd_ptr;
wire  [AW:0] level = wr_ptr - rd_ptr;

always_comb begin
    empty     = (level == 0);
    full      = (level >= FIFO_FULL);
    half_full = (level > FIFO_HALF) && !full;
    endata    = ca_en;
end

always_ff @(posedge clk) data_out <= mem[rd_ptr[AW-1:0]];

// ---------------------------------------------------------------------------
// Boxcar average of the 48 kHz source between two sample strobes.
// ---------------------------------------------------------------------------
wire signed [15:0] in_l = src_active ? src_l : 16'sd0;
wire signed [15:0] in_r = src_active ? src_r : 16'sd0;

logic signed [20:0] acc_l, acc_r;
logic        [3:0]  acc_n;
logic signed [20:0] snap_l, snap_r;
logic        [3:0]  snap_n;
logic signed [15:0] last_l, last_r, avg_l, avg_r;
logic signed [38:0] prod_l, prod_r;

// 65536 / n, with n = 0 handled separately.
function automatic [16:0] recip(input [3:0] n);
    case (n)
        4'd1:  recip = 17'd65536;
        4'd2:  recip = 17'd32768;
        4'd3:  recip = 17'd21845;
        4'd4:  recip = 17'd16384;
        4'd5:  recip = 17'd13107;
        4'd6:  recip = 17'd10923;
        4'd7:  recip = 17'd9362;
        4'd8:  recip = 17'd8192;
        4'd9:  recip = 17'd7282;
        4'd10: recip = 17'd6554;
        4'd11: recip = 17'd5958;
        4'd12: recip = 17'd5461;
        4'd13: recip = 17'd5041;
        4'd14: recip = 17'd4681;
        default: recip = 17'd4369;
    endcase
endfunction

// ---------------------------------------------------------------------------
// Packer: one averaged sample becomes 1, 2 or 4 bytes in the FIFO.
// ---------------------------------------------------------------------------
typedef enum logic [2:0] {S_IDLE, S_MUL, S_AVG, S_WRITE} state_t;
state_t state;
logic [1:0] step;

wire        is16   = fmt && !lc;
wire  [2:0] nbytes = (sm ? 3'd2 : 3'd1) << is16;
wire [AW:0] room   = FIFO_SIZE[AW:0] - level;

// Mono is the mean of both channels.
wire signed [16:0] mono_sum = avg_l + avg_r;
wire signed [15:0] mono     = mono_sum[16:1];

wire signed [15:0] pick = sm ? ((is16 ? step[1] : step[0]) ? avg_r : avg_l) : mono;
wire signed [16:0] rnd  = pick + 17'sd128;
wire        [7:0]  pcm8 = (rnd > 17'sd32767) ? 8'h7F : rnd[15:8];
wire        [7:0]  wbyte = is16 ? (step[0] ? pick[15:8] : pick[7:0]) : (pcm8 ^ 8'h80);

// Any change of the capture mode restarts the stream on a sample boundary.
logic [3:0] mode_q;
wire        flush = rst || !cen || (mode_q != {cen, sm, fmt, lc});

logic [4:0] dbg_hold;
logic [1:0] dbg_ovr, dbg_rde;
logic       dbg_started, dbg_prev_hold;
assign dbg = {dbg_hold, 4'd0, dbg_ovr, dbg_rde};

always_ff @(posedge clk) begin
    ca_en  <= 1'b0;
    mode_q <= {cen, sm, fmt, lc};

    if (rst == 1'b1) begin
        audio_delay <= audio_dev;
    end else if (cen == 1'b0) begin
        audio_delay <= audio_dev;
    end else if (audio_delay == 0) begin
        audio_delay <= audio_dev;
        ca_en <= 1'b1;
    end else begin
        audio_delay <= audio_delay - 1'd1;
    end

    // Accumulate the source; a strobe snapshots and restarts the window.
    if (ca_en) begin
        snap_l <= acc_l;
        snap_r <= acc_r;
        snap_n <= acc_n;
        acc_l  <= src_ce ? 21'(in_l) : 21'sd0;
        acc_r  <= src_ce ? 21'(in_r) : 21'sd0;
        acc_n  <= src_ce ? 4'd1 : 4'd0;
    end else if (src_ce) begin
        acc_l <= acc_l + 21'(in_l);
        acc_r <= acc_r + 21'(in_r);
        if (acc_n != 4'd15) acc_n <= acc_n + 1'd1;
    end

    case (state)
        S_IDLE: begin
            if (ca_en) state <= S_MUL;
        end
        S_MUL: begin
            prod_l <= snap_l * $signed({1'b0, recip(snap_n)});
            prod_r <= snap_r * $signed({1'b0, recip(snap_n)});
            state  <= S_AVG;
        end
        S_AVG: begin
            // No source sample in this window: repeat the previous value.
            avg_l  <= (snap_n == 0) ? last_l : 16'((prod_l + 39'sd32768) >>> 16);
            avg_r  <= (snap_n == 0) ? last_r : 16'((prod_r + 39'sd32768) >>> 16);
            last_l <= (snap_n == 0) ? last_l : 16'((prod_l + 39'sd32768) >>> 16);
            last_r <= (snap_n == 0) ? last_r : 16'((prod_r + 39'sd32768) >>> 16);
            step   <= 2'd0;
            // Count gaps of the source stream (episodes), but not the start-up before the first sample.
            if (snap_n != 0) dbg_started <= 1'b1;
            dbg_prev_hold <= (snap_n == 0);
            if (snap_n == 0 && !dbg_prev_hold && dbg_started && dbg_hold != 5'd31) dbg_hold <= dbg_hold + 1'd1;
            if (room < nbytes && dbg_ovr != 2'd3) dbg_ovr <= dbg_ovr + 1'd1;
            // A sample is stored completely or dropped (overrun).
            if (room >= nbytes) state <= S_WRITE;
            else                state <= S_IDLE;
        end
        S_WRITE: begin
            mem[wr_ptr[AW-1:0]] <= wbyte;
            wr_ptr <= wr_ptr + 1'd1;
            step   <= step + 1'd1;
            if ({1'b0, step} + 3'd1 == nbytes) state <= S_IDLE;
        end
        default: state <= S_IDLE;
    endcase

    if (rd && !empty) rd_ptr <= rd_ptr + 1'd1;
    if (rd && empty && dbg_rde != 2'd3) dbg_rde <= dbg_rde + 1'd1;

    if (flush) begin
        wr_ptr <= '0;
        rd_ptr <= '0;
        state  <= S_IDLE;
        dbg_hold <= '0;
        dbg_ovr  <= '0;
        dbg_rde  <= '0;
        dbg_started <= 1'b0;
        dbg_prev_hold <= 1'b0;
        last_l <= 16'sd0;
        last_r <= 16'sd0;
        acc_l  <= 21'sd0;
        acc_r  <= 21'sd0;
        acc_n  <= 4'd0;
    end
end
endmodule
