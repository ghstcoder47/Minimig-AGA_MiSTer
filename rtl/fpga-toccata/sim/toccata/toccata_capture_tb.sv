// Testbench for toccata_capture: a 48 kHz ramp is recorded in every format.
// Run: iverilog -g2012 -o /tmp/cap_tb toccata_capture_tb.sv ../../toccata_capture.sv && vvp /tmp/cap_tb
`timescale 1ns/1ps

module toccata_capture_tb;
    localparam CLK = 28_359_380;

    logic clk = 0;
    always #17.63 clk = ~clk;

    logic rst = 1, cen = 0, sm = 0, fmt = 0, lc = 0, css = 0;
    logic [2:0] freq_sel = 3'd1; // 16 kHz
    logic src_ce = 0, src_active = 1, rd = 0;
    logic signed [15:0] src_l = 0, src_r = 0;
    logic [7:0] data_out;
    logic empty, half_full, full, endata;

    toccata_capture #(.CLK_FREQUENCY(CLK)) dut (
        .clk, .rst, .cen, .freq_sel, .sm, .fmt, .lc, .css,
        .src_ce, .src_l, .src_r, .src_active,
        .data_out, .rd, .empty, .half_full, .full, .endata
    );

    // 48 kHz source: constant 0x1000 left, -0x2000 right.
    int cnt = 0;
    always @(posedge clk) begin
        src_ce <= 0;
        cnt <= cnt + 1;
        if (cnt >= CLK / 48000) begin
            cnt <= 0;
            src_ce <= 1;
        end
    end

    int errors = 0;
    task automatic check(input string what, input int got, input int exp);
        if (got !== exp) begin
            $display("FAIL %s: got %0d expected %0d", what, got, exp);
            errors++;
        end
    endtask

    task automatic pop(output logic [7:0] b);
        @(posedge clk); b = data_out;
        rd <= 1; @(posedge clk); rd <= 0; @(posedge clk); @(posedge clk);
    endtask

    logic [7:0] b0, b1, b2, b3;

    task automatic run(input bit s, input bit f, input string name);
        cen = 0; sm = s; fmt = f;
        repeat (4) @(posedge clk);
        cen = 1;
        repeat (CLK / 16000 * 6) @(posedge clk);
        if (empty) begin $display("FAIL %s: FIFO stayed empty", name); errors++; end
    endtask

    initial begin
        src_l = 16'sh1000;
        src_r = -16'sh2000;
        repeat (10) @(posedge clk);
        rst = 0;

        // 8 bit mono: mean of L and R = -0x0800 -> 0x80 - 8 = 0x78
        run(0, 0, "mono8");
        pop(b0);
        check("mono8", b0, 8'h78);

        // 16 bit mono: little endian -0x0800 = 0xF800
        run(0, 1, "mono16");
        pop(b0); pop(b1);
        check("mono16 lsb", b0, 8'h00);
        check("mono16 msb", b1, 8'hF8);

        // 8 bit stereo: L 0x10 -> 0x90, R -0x20 -> 0x60
        run(1, 0, "stereo8");
        pop(b0); pop(b1);
        check("stereo8 L", b0, 8'h90);
        check("stereo8 R", b1, 8'h60);

        // 16 bit stereo: L 0x1000, R 0xE000
        run(1, 1, "stereo16");
        pop(b0); pop(b1); pop(b2); pop(b3);
        check("stereo16 Llsb", b0, 8'h00);
        check("stereo16 Lmsb", b1, 8'h10);
        check("stereo16 Rlsb", b2, 8'h00);
        check("stereo16 Rmsb", b3, 8'hE0);

        // Inactive source records silence.
        src_active = 0;
        run(0, 0, "silence");
        pop(b0);
        check("silence", b0, 8'h80);
        src_active = 1;

        // Never read: the FIFO fills, then reports full without wrapping.
        run(1, 1, "overrun");
        repeat (CLK / 16000 * 600) @(posedge clk);
        if (!full) begin $display("FAIL overrun: not full"); errors++; end
        cen = 0;
        repeat (4) @(posedge clk);
        if (!empty) begin $display("FAIL: cen low must flush"); errors++; end

        if (errors == 0) $display("PASS");
        else $display("%0d errors", errors);
        $finish;
    end
endmodule
