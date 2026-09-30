// Generic tristates that synth_gowin -noiopads leaves in submodules.
// Gowin primitive models come directly from the pinned Yosys package.
module \$_TBUF_ (input A, input E, output Y);
    assign Y = E ? A : 1'bz;
endmodule
