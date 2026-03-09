// SDRAM Controller for Tang Primer 25K External SDRAM Module
//
// The dock's SDRAM module has two MT48LC16M16A2 chips (32MB each = 64MB total)
// sharing a 16-bit data bus. A single CS pin selects between chips:
// CS=LOW selects chip 0, CS=HIGH selects chip 1 (module has onboard inverter).
//
// Per chip: 4 banks, 8192 rows (13-bit), 512 columns (9-bit), 16-bit data
// CAS latency = 3, burst length = 4 (4 × 16-bit = 64 bits = 8 bytes per burst)
//
// Memory data is stored interleaved to optimize SPI FLASH emulation:
// Within each 8-byte burst (4 × 16-bit words):
//   Word 0: dq[7:0] = bit 7 of each byte, dq[15:8] = bit 6 of each byte
//   Word 1: dq[7:0] = bit 5 of each byte, dq[15:8] = bit 4 of each byte
//   Word 2: dq[7:0] = bit 3 of each byte, dq[15:8] = bit 2 of each byte
//   Word 3: dq[7:0] = bit 1 of each byte, dq[15:8] = bit 0 of each byte
// This ensures the MSB of every byte is always received first from SDRAM.
//
// Address mapping (23-bit burst address, 8 bytes per burst = 64MB):
//   spi_addr[22]    = chip select (0=chip0, 1=chip1)
//   spi_addr[21:9]  = row (13 bits)
//   spi_addr[8:7]   = bank (2 bits)
//   spi_addr[6:0]   = column / 4 (7 bits, col[8:2], burst-4 aligned)

`default_nettype none

module sdram(
    input wire clk,
    input wire aux_clk,      // Phase-shifted clock for read data capture
    input wire reset,

    // SDRAM physical interface (directly to IODELAY / pins)
    output reg [1:0] ba_o,
    output reg [12:0] a_o,       // 13-bit address for MT48LC16M16A2
    output reg cs_o,             // Single CS: LOW=chip0, HIGH=chip1
    output reg ras_o,
    output reg cas_o,
    output reg we_o,
    output reg [15:0] dq_o,
    output reg [1:0] dqm_o,
    input wire [15:0] dq_i,
    output reg dq_oe_o,
    
    // Control signals from spi_trx (directly from SPI clock domain)
    input wire spi_inhibit_refresh,
    input wire spi_cmd_activate,
    input wire spi_cmd_read,
    input wire [22:0] spi_addr,      // 23-bit burst address (64MB)

    // Control signals from glue (serial path)
    input wire [1:0] access_cmd,     // 00=nop 01=read 10=write 11=activate
    input wire [24:0] access_addr,   // Access address (25-bit: chip + row + bank + col)
    input wire inhibit_refresh,
    output reg cmd_busy,

    output reg [63:0] read_buffer,
    output reg read_busy,

    input wire [63:0] write_buffer
);

    parameter CLK_FREQ_MHZ = 132;
    parameter BURST_LEN = 4;

    // Timing parameters (in clock cycles at ~132MHz)
    localparam integer tINIT        = 100 * CLK_FREQ_MHZ;   // 100us init
    localparam integer tREFRESH     = (CLK_FREQ_MHZ * 32000) / 8192;  // ~516 cycles
    localparam integer tRP          = 3;   // 18ns precharge (min 15ns)
    localparam integer tRC          = 9;   // 60ns row cycle
    localparam integer tMRD         = 2;   // 2 cycles mode register set
    localparam integer tRCD         = 3;   // 18ns RAS to CAS delay (min 15ns)
    localparam integer tDPL         = 2;   // Write recovery
    localparam integer tRAS         = 6;   // 37ns row active time
    localparam integer tCAS         = 3;   // CAS latency = 3 for external SDRAM
    
    // Derived timing
    localparam integer tREAD  = tCAS + BURST_LEN + 1;
    localparam integer tWRITE = BURST_LEN + tDPL + tRP;

    // State machine states
    localparam
        STA_INIT            = 0,
        STA_INIT_PRECHARGE  = 1,
        STA_INIT_REFRESH    = 2,
        STA_IDLE            = 3,
        STA_SETMODE         = 4,
        STA_REFRESH         = 5,
        STA_ACTIVATE        = 6,
        STA_READ            = 7,
        STA_WRITE           = 8,
        STA_INIT_CHIP2      = 9,   // Init second chip
        STA_INIT_PRECHARGE2 = 10,
        STA_INIT_REFRESH2   = 11,
        STA_SETMODE2        = 12,
        STA_REFRESH2        = 13;  // Refresh second chip

    // Burst mode encoding
    localparam [2:0] BURST_MODE =
        (BURST_LEN == 1) ? 3'b000 :
        (BURST_LEN == 2) ? 3'b001 :
        (BURST_LEN == 4) ? 3'b010 :
        (BURST_LEN == 8) ? 3'b011 :
        3'b111;

    reg [3:0] state;
    reg [$clog2(tINIT)-1:0] initcount;
    reg initrefreshcount;
    reg [4:0] cmdcount;
    reg [4:0] cmdtarget;
    reg [$clog2(tREFRESH):0] refreshcount;
    
    // Track which chip we're refreshing (alternate between chips)
    reg refresh_chip;
    
    // Buffer control signals from SPI domain
    reg [1:0] spi_inhibit_refresh_buf;
    reg [1:0] spi_cmd_activate_buf;
    reg [1:0] spi_cmd_read_buf;
    
    reg spi_cmd_activate_ack;
    reg spi_cmd_read_ack;
    
    wire do_inhibit_refresh = (spi_inhibit_refresh_buf[1] || inhibit_refresh);
    
    // Address decoding for MT48LC16M16A2 (2 × 32MB = 64MB)
    // SPI path: 23-bit burst address
    //   [22]    = chip select
    //   [21:9]  = row (13 bits)
    //   [8:7]   = bank (2 bits)
    //   [6:0]   = column burst index (col[8:2])
    wire spi_chip_sel  = spi_addr[22];
    wire [12:0] spi_row  = spi_addr[21:9];
    wire [1:0]  spi_bank = spi_addr[8:7];
    wire [8:0]  spi_col  = {spi_addr[6:0], 2'b00};  // Burst-4 aligned

    // Serial/glue path: 25-bit address = {chip, row, bank, col}
    //   [24]     = chip select
    //   [23:11]  = row (13 bits)
    //   [10:9]   = bank (2 bits)
    //   [8:0]    = column (9 bits, low 2 bits = burst offset)
    wire access_chip_sel  = access_addr[24];
    wire [12:0] access_row  = access_addr[23:11];
    wire [1:0]  access_bank = access_addr[10:9];
    wire [8:0]  access_col  = access_addr[8:0];

    reg [4:0] readcount;
    reg [1:0] rdbuf_write_ptr;
    reg [2:0] wrbuf_read_ptr;

    // Read data captured on aux_clk domain, then transferred
    reg [15:0] dq_captured;
    reg [15:0] dq_captured_sync;
    
    // Capture DQ on aux_clk (phase-shifted for proper sampling)
    always @(posedge aux_clk) begin
        dq_captured <= dq_i;
    end
    
    // Synchronize captured data to main clock domain
    always @(posedge clk) begin
        dq_captured_sync <= dq_captured;
    end

    integer i;

    always @(posedge clk) begin
        if (reset) begin
            state <= STA_INIT;
            cs_o <= 0;           // Select chip 0 for init (CS LOW = chip 0)
            ras_o <= 1;
            cas_o <= 1;
            we_o <= 1;
            ba_o <= 0;
            a_o <= 0;
            dq_oe_o <= 0;
            dq_o <= 0;
            dqm_o <= 2'b11;

            cmd_busy <= 1;

            initcount <= 0;
            initrefreshcount <= 0;
            cmdcount <= 0;
            cmdtarget <= 0;
            refreshcount <= 0;
            refresh_chip <= 0;
            
            read_buffer <= 0;
            read_busy <= 0;
            readcount <= 0;

            rdbuf_write_ptr <= 0;
            wrbuf_read_ptr <= 0;
            
            spi_inhibit_refresh_buf <= 0;
            spi_cmd_activate_buf <= 0;
            spi_cmd_read_buf <= 0;
            
            spi_cmd_activate_ack <= 0;
            spi_cmd_read_ack <= 0;
        end
        else begin
            refreshcount <= refreshcount + 1;
            
            // Synchronize SPI control signals
            spi_inhibit_refresh_buf <= {spi_inhibit_refresh_buf[0], spi_inhibit_refresh};
            spi_cmd_activate_buf <= {spi_cmd_activate_buf[0], spi_cmd_activate};
            spi_cmd_read_buf <= {spi_cmd_read_buf[0], spi_cmd_read};
            
            if (spi_cmd_activate_ack && !spi_cmd_activate_buf[1]) spi_cmd_activate_ack <= 0;
            if (spi_cmd_read_ack && !spi_cmd_read_buf[1]) spi_cmd_read_ack <= 0;

            // Update busy flag
            // Note: STA_REFRESH and STA_REFRESH2 are listed explicitly to
            // prevent a one-cycle cmd_busy dip when cmdcount reaches
            // cmdtarget-1 during the REFRESH->REFRESH2 handoff.  Without
            // these terms the dispatch block could accept a serial command
            // that is then silently overwritten by the REFRESH2 transition
            // (Verilog last-write-wins), causing the SDRAM to not drive DQ.
            cmd_busy <= (state <= STA_INIT_REFRESH) ||
                        (state >= STA_INIT_CHIP2 && state <= STA_SETMODE2) ||
                        ((state != STA_IDLE) && (cmdcount < cmdtarget-1)) ||
                        (state == STA_REFRESH) ||
                        (state == STA_REFRESH2) ||
                        (access_cmd != 2'b00) ||
                        ((refreshcount >= tREFRESH-1) && !do_inhibit_refresh);

            if (state == STA_INIT) begin
                // Wait for SDRAM power-up (100us) - chip 0 selected (CS LOW)
                if (initcount >= tINIT) begin
                    state <= STA_INIT_PRECHARGE;
                    cmdcount <= 1;
                    cmdtarget <= tRP;

                    cs_o <= 0;       // Chip 0 (CS LOW)
                    ras_o <= 0;
                    cas_o <= 1;
                    we_o <= 0;       // PRECHARGE
                    dqm_o <= 2'b11;
                    a_o[10] <= 1;    // All banks
                end
                else begin
                    initcount <= initcount + 1;
                    // NOP
                    ras_o <= 1;
                    cas_o <= 1;
                    we_o <= 1;
                end
            end
            else if ((state != STA_IDLE) && (cmdcount < cmdtarget)) begin
                // Waiting for command to complete
                // Issue NOP
                ras_o <= 1;
                cas_o <= 1;
                we_o <= 1;

                if (state == STA_WRITE) begin
                    if (cmdcount < BURST_LEN) begin
                        // Feed write data (16-bit at a time, interleaved)
                        dq_oe_o <= 1;
                        
                        // Interleaved data layout for 16-bit bus:
                        // wrbuf_read_ptr counts 0..3 (4 words per burst)
                        // Each word carries 2 bits per byte
                        for (i = 0; i < 8; i = i + 1) begin
                            dq_o[i]   <= write_buffer[i*8 + 7 - wrbuf_read_ptr*2];
                            dq_o[i+8] <= write_buffer[i*8 + 6 - wrbuf_read_ptr*2];
                        end
                        dqm_o <= 2'b00;

                        wrbuf_read_ptr <= wrbuf_read_ptr + 1;
                    end
                    else begin
                        dq_oe_o <= 0;
                        dqm_o <= 2'b11;
                    end
                end

                cmdcount <= cmdcount + 1;
            end
            else begin
                // No command running, determine next command
                cmdcount <= 1;

                if (state == STA_INIT_PRECHARGE) begin
                    state <= STA_INIT_REFRESH;
                    cmdtarget <= tRC;
                    initrefreshcount <= 0;

                    // REFRESH command (chip 0)
                    cs_o <= 0;
                    ras_o <= 0;
                    cas_o <= 0;
                    we_o <= 1;
                end
                else if (state == STA_INIT_REFRESH) begin
                    if (initrefreshcount == 1) begin
                        state <= STA_SETMODE;
                        cmdtarget <= tMRD;
                        refreshcount <= 1;

                        // MODE REGISTER SET (chip 0)
                        cs_o <= 0;
                        ras_o <= 0;
                        cas_o <= 0;
                        we_o <= 0;
                        dqm_o <= 2'b11;
                        ba_o <= 2'b00;
                        a_o <= 0;
                        a_o[9] <= 1'b0;         // Write burst: programmed length
                        a_o[8:7] <= 2'b00;      // Standard operation
                        a_o[6:4] <= tCAS;       // CAS latency = 3
                        a_o[3] <= 1'b0;         // Burst type: sequential
                        a_o[2:0] <= BURST_MODE; // Burst length = 4
                    end
                    else begin
                        initrefreshcount <= 1;

                        // Another REFRESH (chip 0)
                        cs_o <= 0;
                        ras_o <= 0;
                        cas_o <= 0;
                        we_o <= 1;
                        dqm_o <= 2'b11;
                    end
                end
                else if (state == STA_SETMODE) begin
                    // Chip 0 init done, now init chip 1
                    state <= STA_INIT_CHIP2;
                    cmdcount <= 1;
                    cmdtarget <= tRP;

                    // PRECHARGE all banks on chip 1
                    cs_o <= 1;       // Chip 1 (CS HIGH)
                    ras_o <= 0;
                    cas_o <= 1;
                    we_o <= 0;
                    dqm_o <= 2'b11;
                    a_o <= 0;
                    a_o[10] <= 1;    // All banks
                end
                else if (state == STA_INIT_CHIP2) begin
                    // Precharge done for chip 1, do refresh
                    state <= STA_INIT_REFRESH2;
                    cmdtarget <= tRC;
                    initrefreshcount <= 0;

                    // REFRESH command (chip 1)
                    cs_o <= 1;
                    ras_o <= 0;
                    cas_o <= 0;
                    we_o <= 1;
                end
                else if (state == STA_INIT_REFRESH2) begin
                    if (initrefreshcount == 1) begin
                        state <= STA_SETMODE2;
                        cmdtarget <= tMRD;

                        // MODE REGISTER SET (chip 1)
                        cs_o <= 1;
                        ras_o <= 0;
                        cas_o <= 0;
                        we_o <= 0;
                        dqm_o <= 2'b11;
                        ba_o <= 2'b00;
                        a_o <= 0;
                        a_o[9] <= 1'b0;
                        a_o[8:7] <= 2'b00;
                        a_o[6:4] <= tCAS;
                        a_o[3] <= 1'b0;
                        a_o[2:0] <= BURST_MODE;
                    end
                    else begin
                        initrefreshcount <= 1;

                        // Another REFRESH (chip 1)
                        cs_o <= 1;
                        ras_o <= 0;
                        cas_o <= 0;
                        we_o <= 1;
                        dqm_o <= 2'b11;
                    end
                end
                else if (state == STA_SETMODE2) begin
                    // Both chips initialized, go to idle
                    state <= STA_IDLE;
                    cs_o <= 0;
                    ras_o <= 1;
                    cas_o <= 1;
                    we_o <= 1;
                    dqm_o <= 2'b11;
                end
                else if (spi_cmd_activate_buf[1] && !spi_cmd_activate_ack) begin
                    // SPI fast-path activate
                    state <= STA_ACTIVATE;
                    cmdtarget <= tRCD;
                    spi_cmd_activate_ack <= 1;

                    // ACTIVATE command
                    cs_o <= spi_chip_sel;
                    ras_o <= 0;
                    cas_o <= 1;
                    we_o <= 1;
                    dqm_o <= 2'b11;
                    ba_o <= spi_bank;
                    a_o <= spi_row;
                end
                else if (spi_cmd_read_buf[1] && !spi_cmd_read_ack) begin
                    // SPI fast-path read
                    state <= STA_READ;
                    cmdtarget <= tREAD;
                    read_busy <= 1;
                    spi_cmd_read_ack <= 1;

                    // READ command with auto-precharge
                    cs_o <= spi_chip_sel;
                    ras_o <= 1;
                    cas_o <= 0;
                    we_o <= 1;
                    ba_o <= spi_bank;
                    a_o <= 0;
                    a_o[8:0] <= spi_col;     // 9-bit column, burst-4 aligned
                    a_o[10] <= 1;            // Auto precharge
                    dq_oe_o <= 0;
                    dqm_o <= 2'b00;
                end
                else if (access_cmd == 2'b11) begin
                    // Serial path activate
                    state <= STA_ACTIVATE;
                    cmdtarget <= tRCD;

                    // ACTIVATE command
                    cs_o <= access_chip_sel;
                    ras_o <= 0;
                    cas_o <= 1;
                    we_o <= 1;
                    dqm_o <= 2'b11;
                    ba_o <= access_bank;
                    a_o <= access_row;
                end
                else if (access_cmd == 2'b01) begin
                    // Serial path read
                    state <= STA_READ;
                    cmdtarget <= tREAD + 2;
                    read_busy <= 1;

                    // READ command with auto-precharge
                    cs_o <= access_chip_sel;
                    ras_o <= 1;
                    cas_o <= 0;
                    we_o <= 1;
                    ba_o <= access_bank;
                    a_o <= 0;
                    a_o[8:0] <= access_col;  // 9-bit column
                    a_o[10] <= 1;            // Auto precharge
                    dq_oe_o <= 0;
                    dqm_o <= 2'b00;
                end
                else if (access_cmd == 2'b10) begin
                    // Serial path write
                    state <= STA_WRITE;
                    cmdtarget <= tWRITE;
                    wrbuf_read_ptr <= 1;

                    // WRITE command with auto-precharge
                    cs_o <= access_chip_sel;
                    ras_o <= 1;
                    cas_o <= 0;
                    we_o <= 0;
                    ba_o <= access_bank;
                    a_o <= 0;
                    a_o[8:0] <= access_col;  // 9-bit column
                    a_o[10] <= 1;            // Auto precharge
                    dq_oe_o <= 1;
                    
                    // First write data word (interleaved for 16-bit bus)
                    for (i = 0; i < 8; i = i + 1) begin
                        dq_o[i]   <= write_buffer[i*8 + 7];
                        dq_o[i+8] <= write_buffer[i*8 + 6];
                    end
                    dqm_o <= 2'b00;
                end
                else if ((refreshcount >= tREFRESH) && !do_inhibit_refresh) begin
                    // Auto refresh - alternate between chips
                    state <= STA_REFRESH;
                    cmdtarget <= tRC;
                    refreshcount <= 1;

                    // REFRESH command for current chip
                    cs_o <= refresh_chip;
                    ras_o <= 0;
                    cas_o <= 0;
                    we_o <= 1;
                    dqm_o <= 2'b11;
                end
                else begin
                    state <= STA_IDLE;
                    ras_o <= 1;
                    cas_o <= 1;
                    we_o <= 1;
                    dqm_o <= 2'b11;
                end
            end

            // After refresh completes, schedule refresh for the other chip
            if (state == STA_REFRESH && cmdcount >= cmdtarget) begin
                if (!refresh_chip) begin
                    // Just refreshed chip 0, now refresh chip 1
                    state <= STA_REFRESH2;
                    cmdcount <= 1;
                    cmdtarget <= tRC;
                    
                    cs_o <= 1;       // Chip 1
                    ras_o <= 0;
                    cas_o <= 0;
                    we_o <= 1;
                    dqm_o <= 2'b11;
                    
                    refresh_chip <= 1;
                end
                else begin
                    refresh_chip <= 0;
                end
            end
            
            if (state == STA_REFRESH2 && cmdcount >= cmdtarget) begin
                refresh_chip <= 0;
            end
            
            // Read data capture (de-interleave 16-bit data to 64-bit buffer)
            // Uses aux_clk-captured data for proper timing
            if ((readcount > tCAS) && (readcount <= tCAS + BURST_LEN)) begin
                // Capture read data and de-interleave from 16-bit bus
                // rdbuf_write_ptr counts 0..3 (4 words in burst)
                for (i = 0; i < 8; i = i + 1) begin
                    read_buffer[i*8 + 7 - rdbuf_write_ptr*2] <= dq_captured_sync[i];
                    read_buffer[i*8 + 6 - rdbuf_write_ptr*2] <= dq_captured_sync[i+8];
                end
                
                if (rdbuf_write_ptr == BURST_LEN - 1) read_busy <= 0;
                rdbuf_write_ptr <= rdbuf_write_ptr + 1;
            end
            else
                rdbuf_write_ptr <= 0;
            
            readcount <= (state == STA_READ) ? cmdcount : 0;
        end
    end

endmodule
