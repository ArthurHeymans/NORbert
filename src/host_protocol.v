// Host protocol, transport selection, configuration and TOCTOU control.
// SDRAM requests use the host client port; program/RMW is a separate owner.
//
// Ported for Tang Primer 25K with 64MB external SDRAM (23-bit burst addresses)
// Serial path uses 25-bit access_addr = {chip, row[12:0], bank[1:0], col[8:0]}
//
// Accepts bytes from EITHER UART or FT245, each through its own receive
// FIFO.  When idle (no active command), whichever port has a byte first
// becomes the "active port" for that entire command; bytes from the other
// port wait in their FIFO until the command ends.  Responses are routed
// back to the same port.
//
// A byte is only popped when it can be processed, never dropped: bytes of
// the SDRAM/configuration commands (RAMREAD, RAMWRITE, CHIPCONFIG) wait
// while the SPI bus is selected or the program engine owns SDRAM. All
// other commands (including HOLDCTL, LOGCTL and TOCTOU and their argument
// bytes) touch neither and are processed regardless of the SPI bus.
//
// SPI emulation is gated by `spi_running`, controlled by the START (0x34)
// and STOP (0x35) serial commands.  At power-on/reset the FPGA starts in
// STOPPED state, which means spi_trx is held in reset via top.v (so SPI
// pin state never blocks serial traffic) and all serial commands are
// accepted.  When RUNNING, the SDRAM/configuration commands (RAMREAD,
// RAMWRITE, CHIPCONFIG) are rejected until the host sends STOP; all other
// commands keep working so the host can reach NORbert regardless of what
// the emulated master is doing.

`default_nettype none

module host_protocol(
    input wire clk,
    input wire reset,

    // UART byte interface
    input wire rxd_strobe,
    input wire [7:0] rxd_data,

    input wire txd_ready,
    output reg txd_strobe,
    output reg [7:0] txd_data,

    // FT245 byte interface (pull-based via RX FIFO)
    input wire ft_rx_data_available,
    input wire [7:0] ft_rx_data,
    output reg ft_rx_pop,

    input wire ft_txd_ready,
    output reg ft_txd_strobe,
    output reg [7:0] ft_txd_data,

    // SDRAM control signals
    output wire [1:0] sdram_access_cmd, // 00=nop 01=read 10=write 11=activate
    output wire [24:0] sdram_access_addr,  // 25-bit: {chip, row, bank, col}
    output reg sdram_inhibit_refresh,
    input wire sdram_cmd_busy,
    input wire sdram_access_accept, // A posted command was actually dispatched

    input wire [63:0] sdram_read_buffer,
    input wire sdram_read_busy,

    output wire [63:0] sdram_write_buffer,
    
    // SPI signals
    input wire spi_reset,
    input wire spi_csel,
    
    input wire spi_writing,
    input wire pp_init_done, pp_init_finishing,
    output wire program_allowed,
    output reg [3:0] trap_triggered,
    
    
    // Chip configuration outputs (from CHIPCONFIG command)
    output reg [23:0] cfg_jedec_id,
    output reg cfg_4byte,
    output reg [22:0] cfg_chip_erase_bursts,
    
    // SFDP table read interface (for spi_trx)
    input wire spi_clk,              // SPI clock: read-port clock for the SFDP BSRAM
    input wire [6:0] sfdp_raddr,
    output wire [7:0] sfdp_rdata,     // Synchronous BSRAM read, invalid bytes read as 0xFF
    
    // SPI emulation enable (1 = running, 0 = stopped).
    // Exported to top.v which uses it to gate spi_reset feeding both
    // spi_trx and back into this module -- so while stopped the SPI pins
    // are ignored and serial commands always have a clear path.
    output wire spi_running,

    // Target flash HOLD control (active high: 1 = assert #HOLD on target)
    output reg hold_out,

    // Logging control: 1 = logger captures SPI events into the ring
    // FIFO; 0 = logger ignores events.  The host drains the FIFO via
    // CMD_LOGPOLL regardless of this flag (so residual bytes remain
    // retrievable after LOGCTL stop).
    output reg log_active,

    // Logger ring FIFO read interface -- glue pops bytes here while
    // serving a CMD_LOGPOLL response.
    input wire log_fifo_data_available,
    input wire [7:0] log_fifo_read_data,
    output reg log_fifo_read_strobe,

    // TOCTOU trap interface
    // Structured log inputs (synchronized from SPI domain)
    input wire log_addr_valid_sync, // Pulse: address phase complete (system clock)
    input wire [23:0] log_addr_sync,// Flash byte address (latched when valid)
    input wire spi_active_sync,     // SPI CS active (synchronized)

    // SDRAM prefetch diagnostics (spi_prefetch, SPI clock domain). These
    // are per-transaction flags, cleared at the next transaction's first
    // clock; glue latches them so a host read still sees them afterwards.
    input wire prefetch_underrun,
    input wire prefetch_thin,

    // TOCTOU redirect outputs (directly to address mux in top.v)
    output reg redirect_active,
    // The trap comparison for the previous cycle's address event is still
    // running. With that event cycle (in top.v) this covers the whole time
    // the redirect decision is not final.
    output reg trap_check_pending,
    output reg [22:0] redirect_mask,    // Burst address mask bits
    output reg [22:0] redirect_base,    // Burst address replacement base

    // TOCTOU trap notification (to logger)
    output reg trap_notify_strobe,
    output reg [1:0] trap_notify_index,
    output reg [23:0] trap_notify_addr,

    output reg [7:0] led
);

    `include "host_protocol.vh"

    // Max log bytes sent per poll.  Bounds response length so the host
    // cannot be starved by a busy SPI master filling the FIFO faster
    // than it can be drained.  Remaining data is delivered on the next
    // poll.
    localparam [7:0] LOG_POLL_MAX = 8'd255;

    reg [7:0] cmd;
    reg [7:0] in_count;
    
    // Idle timeout: reset the parser if no byte arrives for a while in the
    // middle of a multi-byte command. Handles spurious bytes from the
    // USB-UART bridge on port open/close. Headers time out after 2^16
    // cycles (~546us). A RAMWRITE payload gets 2^22 cycles (~35ms): after
    // a timeout the remaining payload bytes would be parsed as commands,
    // so a USB stall inside a block must not trigger it. Hosts wait longer
    // than that before resynchronizing (tool/src/protocol.rs).
    reg [22:0] serial_idle_count;
    wire serial_idle_expired = (cmd == CMD_RAMWRITE && in_count > 8'd5) ?
                               serial_idle_count[22] : serial_idle_count[16];

    reg [22:0] addr;       // 23-bit burst address
    reg [15:0] len;        // 16-bit burst count (v3: 2 header bytes)

    reg [2:0] read_state;
    reg [2:0] read_pos;

    reg [2:0] write_state;
    reg [2:0] write_pos;

    // CMD_LOGPOLL state machine.
    //   log_poll_state 0 = idle
    //                  1 = drain FIFO (send/escape log bytes while available)
    //                  2 = terminator (send 0xA0 and return to idle)
    //                  3 = escaped byte payload
    reg [1:0] log_poll_state;
    reg [7:0] log_poll_remain;       // raw log bytes still allowed before terminator
    reg [7:0] log_poll_escape_code;  // second byte after LOG_POLL_ESCAPE

    reg txd_strobe_buf;
    reg [7:0] txd_data_buf;
    
    reg rxd_strobe_buf;
    reg [7:0] rxd_data_buf;

    wire sdram_busy = (sdram_access_cmd != 0) || sdram_cmd_busy || sdram_read_busy;
    
    reg [63:0] write_buffer;
    reg write_strobe;
    reg write_strobe_r;  // delayed copy for rising-edge detection
    
    
    reg [1:0] spi_csel_buf;

    // -----------------------------------------------------------
    // SDRAM prefetch fault latch
    //
    // spi_prefetch's flags live in the SPI clock domain and stay high until
    // the next transaction. Synchronize them, then latch each rising edge:
    // latching levels would re-report the same fault after a host read while
    // the SPI transaction (or the gap before the next one) is still active.
    // An address phase leaves plenty of clocks with the flags low before a
    // subsequent transaction can raise them again.
    //
    // The synchronizer and edge history keep tracking through reset: a flag
    // still high from before a reset is then an old level, not a new edge,
    // so reset clears a latched fault without it coming straight back.
    // -----------------------------------------------------------
    reg [1:0] prefetch_faults;
    reg [1:0] prefetch_faults_sync1 = 2'b00;
    reg [1:0] prefetch_faults_sync2 = 2'b00;
    reg [1:0] prefetch_faults_prev = 2'b00;
    wire [1:0] prefetch_faults_event = prefetch_faults_sync2 & ~prefetch_faults_prev;
    wire [1:0] prefetch_faults_now = prefetch_faults | prefetch_faults_event;

    always @(posedge clk) begin
        prefetch_faults_sync1 <= {prefetch_thin, prefetch_underrun};
        prefetch_faults_sync2 <= prefetch_faults_sync1;
        prefetch_faults_prev <= prefetch_faults_sync2;
    end
    
    // Active port selection: 0 = UART, 1 = FT245
    // Latched when a command byte arrives while idle.
    reg active_port;

    wire cmd_idle = (cmd == CMD_NOP) && (in_count == 0) &&
                    (read_state == 3'd0) && (write_state == 3'd0) &&
                    (log_poll_state == 0);
    wire mux_txd_ready = active_port ? ft_txd_ready : txd_ready;

    // SDRAM gate: SDRAM/configuration commands only proceed while the SPI
    // bus is idle and the program engine does not own SDRAM. When
    // spi_running=0, top.v forces this module's spi_reset input high so
    // this gate collapses to just !spi_writing.
    wire serial_gate = (spi_reset || spi_csel_buf[1]) && !spi_writing;

    // Commands that never touch SDRAM, chip configuration or spi_trx state.
    // They and their argument bytes bypass the SDRAM gate.
    function ungated_opcode;
        input [7:0] op;
        begin
            case (op)
            CMD_VERSION, CMD_START, CMD_STOP, CMD_STATUS, CMD_PREFETCH,
            CMD_LOGPOLL, CMD_HOLDCTL, CMD_LOGCTL, CMD_TOCTOU:
                ungated_opcode = 1'b1;
            default:
                ungated_opcode = 1'b0;
            endcase
        end
    endfunction

    // UART receive FIFO. The UART has no flow control, so its bytes are
    // buffered here like the FT245's (in top.v) instead of being dropped
    // while they cannot be processed.
    wire uart_rx_available;
    wire [7:0] uart_rx_data;
    wire uart_rx_space;
    reg uart_rx_pop;
    fifo #(.WIDTH(8), .NUM(16), .FREESPACE(1)) uart_rx_fifo(
        .clk(clk), .reset(reset),
        .write_data(rxd_data), .write_strobe(rxd_strobe && uart_rx_space),
        .space_available(uart_rx_space),
        .data_available(uart_rx_available), .more_available(),
        .read_data(uart_rx_data), .read_strobe(uart_rx_pop));

    // Pull arbitration. Between commands either port may start one; during
    // a command only its port is read, and nothing is read while LOGPOLL
    // drains or a RAMREAD response streams. A byte is popped only if it
    // will be processed: the next one opens or continues an ungated
    // command, or the SDRAM gate is open. Both FIFOs pop with registered
    // strobes, so rx_hold spaces pops two cycles apart (well within the
    // FT245's ~22-cycle byte delivery).
    reg rx_hold;
    wire cmd_ungated = cmd == CMD_HOLDCTL || cmd == CMD_LOGCTL || cmd == CMD_TOCTOU;
    wire rx_parser_ready = !rx_hold && (cmd_idle || (in_count != 0 && read_state == 0));
    wire ft_rx_take = rx_parser_ready && ft_rx_data_available &&
                      (cmd_idle || active_port) &&
                      (serial_gate || (cmd_idle ? ungated_opcode(ft_rx_data) : cmd_ungated));
    wire uart_rx_take = rx_parser_ready && uart_rx_available && !ft_rx_take &&
                        (cmd_idle || !active_port) &&
                        (serial_gate || (cmd_idle ? ungated_opcode(uart_rx_data) : cmd_ungated));

    // Pipeline-aware TX flow control.
    // After txd_strobe_buf fires, it takes 2 cycles for ft245 to latch
    // tx_pending and for ft_txd_ready to go low.  During those 2 cycles
    // mux_txd_ready is stale (still high), so glue would over-send.
    // tx_wait counts down to block sends until back-pressure propagates.
    // The !txd_strobe_buf term blocks the cycle immediately after the
    // send (when txd_strobe_buf is visible but tx_wait hasn't loaded yet).
    // UART is unaffected: its 256-byte FIFO keeps txd_ready high, so the
    // only overhead is a 2-cycle gap between bytes (~17ns, negligible at
    // 2 Mbaud = ~5µs/byte).
    reg [1:0] tx_wait;
    wire can_send = mux_txd_ready && (tx_wait == 0) && !txd_strobe_buf;

    // Heartbeat counter
    reg [25:0] heartbeat;
    

    reg [1:0] host_access_cmd;
    reg [24:0] host_access_addr;
    reg [63:0] host_write_data;
    reg spi_run_requested;
    assign spi_running = spi_run_requested && pp_init_done;

    // A program command can take ownership only between host commands: a
    // multi-byte command's remaining bytes must not wait behind it.
    assign program_allowed = spi_csel_buf[1] && in_count == 0 &&
                             read_state == 0 && write_state == 0 &&
                             !write_strobe && host_access_cmd == 0 &&
                             !sdram_cmd_busy && !sdram_read_busy;
    assign sdram_access_cmd = host_access_cmd;
    assign sdram_access_addr = host_access_addr;
    assign sdram_write_buffer = host_write_data;

    // SFDP table storage: 128 bytes in a dual-clock BSRAM.  A valid
    // length makes unwritten bytes read as 0xFF without a boot scrub
    // competing with CHIPCONFIG for the write port.  Configuration is
    // stable while SPI runs; a zero/shorter table hides old contents.
    reg [7:0] sfdp_mem [0:127];
    reg [6:0] sfdp_waddr;
    reg [7:0] sfdp_wdata;
    reg sfdp_wren;
    reg [7:0] sfdp_valid_len;
    reg [7:0] sfdp_read_byte;
    reg sfdp_read_valid;
    assign sfdp_rdata = sfdp_read_valid ? sfdp_read_byte : 8'hFF;
    
    // CHIPCONFIG state
    reg [6:0] sfdp_wr_pos;
    reg [7:0] cfg_sfdp_remaining;
    
    // TOCTOU trap table (4 entries)
    reg [23:0] trap_start [0:3];     // Byte address match value, pre-masked at SET
    reg [23:0] trap_mask  [0:3];     // Byte address mask (1 = must match)
    reg [23:0] trap_replace [0:3];   // Byte address replacement base
    reg [3:0]  trap_armed;           // Trap is active

    // Separate address comparison from the priority/redirect mux. This
    // adds one system clock, well inside the first (unredirected) SPI
    // burst; subsequent bursts still see the replacement address.
    reg [3:0] trap_match_pending;
    
    // TOCTOU command parsing state
    reg [7:0]  toctou_sub_cmd;
    reg [1:0]  toctou_index;
    reg [23:0] toctou_start_buf;
    reg [23:0] toctou_mask_buf;
    
    // TOCTOU address check: detect rising edge of log_addr_valid_sync
    reg log_addr_valid_prev;
    // spi_trx only signals array reads: SFDP reads use a separate address
    // space, and program/erase addresses are not reads a trap could serve.
    wire log_addr_event = log_addr_valid_sync && !log_addr_valid_prev;
    
    // Convert 23-bit burst address to 25-bit access_addr for SDRAM controller
    // Burst address: [22]=chip, [21:9]=row, [8:7]=bank, [6:0]=col_burst
    // Access addr:   [24]=chip, [23:11]=row, [10:9]=bank, [8:0]=col
    // col = {col_burst[6:0], 2'b00} (burst-4 aligned)
    wire [24:0] addr_to_access = {addr[22], addr[21:9], addr[8:7], addr[6:0], 2'b00};

    integer i;

    // SFDP BSRAM read port (SPI clock domain -- same domain as the
    // sfdp_raddr source and sfdp_rdata consumer in spi_trx, so there is
    // no clock-domain crossing on this path).
    always @(posedge spi_clk) begin
        sfdp_read_byte <= sfdp_mem[sfdp_raddr];
        sfdp_read_valid <= {1'b0, sfdp_raddr} < sfdp_valid_len;
    end

    always @(posedge clk) begin
        if (reset) begin
            cmd <= CMD_NOP;
            in_count <= 0;
            serial_idle_count <= 0;
            addr <= 0;
            len <= 0;

            read_state <= 0;
            read_pos <= 0;

            write_state <= 0;
            write_pos <= 0;

            log_poll_state <= 0;
            log_poll_remain <= 0;
            log_poll_escape_code <= 0;
            log_fifo_read_strobe <= 0;

            host_access_cmd <= 0;
            sdram_inhibit_refresh <= 0;

            write_strobe <= 0;
            write_strobe_r <= 0;

            txd_strobe_buf <= 0;
            txd_data_buf <= 0;
            
            rxd_strobe_buf <= 0;
            rxd_data_buf <= 0;
            
            active_port <= 0;
            ft_rx_pop <= 0;
            uart_rx_pop <= 0;
            rx_hold <= 0;
            ft_txd_strobe <= 0;
            ft_txd_data <= 0;
            
            led <= 0;
            prefetch_faults <= 2'b00;
            
            host_access_addr <= 0;
            host_write_data <= 0;
            write_buffer <= 0;
            
            hold_out <= 0;
            log_active <= 0;
            
            redirect_active <= 0;
            redirect_mask <= 0;
            redirect_base <= 0;
            trap_notify_strobe <= 0;
            
            trap_armed <= 0;
            trap_triggered <= 0;
            toctou_sub_cmd <= 0;
            toctou_index <= 0;
            
            log_addr_valid_prev <= 0;
            trap_match_pending <= 0;
            trap_check_pending <= 0;
            
            cfg_jedec_id <= {8'h17, 8'h40, 8'hEF};  // Default: W25Q64FV (EF 40 17)
            cfg_4byte <= 0;
            cfg_chip_erase_bursts <= 23'h0FFFFF;     // 8MB = 1M bursts - 1
            sfdp_wr_pos <= 0;
            cfg_sfdp_remaining <= 0;

            // Start with SPI emulation STOPPED so the host can always
            // reach the FPGA regardless of target-board state; the host
            // tool sends START explicitly after loading firmware.
            spi_run_requested <= 0;

            for (i = 0; i < 4; i = i + 1) begin
                trap_start[i] <= 0;
                trap_mask[i] <= 0;
                trap_replace[i] <= 0;
            end
            
            tx_wait <= 0;

            sfdp_waddr <= 0;
            sfdp_wdata <= 8'hFF;
            sfdp_wren <= 0;
            sfdp_valid_len <= 0;
        end
        else begin
            txd_strobe_buf <= 0;
            log_fifo_read_strobe <= 0;
            sfdp_wren <= 0;

            // TX pipeline cooldown
            if (txd_strobe_buf)
                tx_wait <= 2'd2;
            else if (tx_wait != 0)
                tx_wait <= tx_wait - 1;

            // Route TX to the active port
            if (active_port) begin
                ft_txd_strobe <= txd_strobe_buf;
                ft_txd_data   <= txd_data_buf;
                txd_strobe    <= 0;
            end else begin
                txd_strobe    <= txd_strobe_buf;
                txd_data      <= txd_data_buf;
                ft_txd_strobe <= 0;
            end

            // Pull-based RX mux (see ft_rx_take/uart_rx_take): FT245 first
            // between commands, otherwise only the command's own port.
            ft_rx_pop <= ft_rx_take;
            uart_rx_pop <= uart_rx_take;
            rx_hold <= ft_rx_take || uart_rx_take;
            rxd_strobe_buf <= ft_rx_take || uart_rx_take;
            if (ft_rx_take) begin
                rxd_data_buf <= ft_rx_data;
                if (cmd_idle) active_port <= 1;
            end else if (uart_rx_take) begin
                rxd_data_buf <= uart_rx_data;
                if (cmd_idle) active_port <= 0;
            end

            // Boot-time START is acknowledged at the final committed scrub.
            if (!pp_init_done && pp_init_finishing && spi_run_requested) begin
                txd_strobe_buf <= 1;
                txd_data_buf <= 8'h01;
            end

            // CHIPCONFIG owns the SFDP write port.
            if (sfdp_wren)
                sfdp_mem[sfdp_waddr] <= sfdp_wdata;

            host_access_addr <= addr_to_access;
            // Snapshot write_buffer into sdram_write_buffer.
            //
            // Problem: FT245 delivers bytes ~12 cycles apart.  After byte 7
            // sets write_strobe, byte 0 of the NEXT burst can arrive and
            // overwrite write_buffer[0] before the SDRAM controller reads
            // sdram_write_buffer.  Continuous mirroring would propagate the
            // corruption.
            //
            // Solution: detect the rising edge of write_strobe (one cycle
            // after byte 7 arrives, so write_buffer has all 8 bytes) and
            // do one final mirror.  After that, freeze until both
            // write_strobe and write_state are clear (write complete).
            // The separate program engine owns its own write payload.
            write_strobe_r <= write_strobe;
            if ((write_strobe && !write_strobe_r) ||
                (!write_strobe && (write_state == 3'd0)))
                host_write_data <= write_buffer;
            
            // Protect only the ACTIVATE -> READ/WRITE pair. Once the
            // auto-precharged burst completes, transport backpressure and
            // BSRAM merging must not block refresh. cmd_busy prevents the
            // next ACTIVATE pulse from racing an already-started refresh.
            sdram_inhibit_refresh <= (read_state == 3'd1 || read_state == 3'd2) ||
                                    (write_state == 3'd1 || write_state == 3'd2);
    
            // Hold the request/address until the controller grants it.
            // busy describes progress, not acceptance; a refresh or another
            // owner can defer dispatch after the caller posts a request.
            if (sdram_access_accept)
                host_access_cmd <= 0;
                
            spi_csel_buf <= {spi_csel_buf[0], spi_csel};
            heartbeat <= heartbeat + 1;

            led[7] <= !spi_reset && !spi_csel_buf[1];  // SPI active
            led[6] <= sdram_cmd_busy;
            led[5] <= spi_writing;
            led[4] <= spi_reset;
            led[3] <= !spi_csel_buf[1];
            led[2] <= hold_out;                         // Target flash held
            led[0] <= heartbeat[25];                    // Heartbeat ~2Hz at 132MHz
            
            // Sticky before the dispatcher below, so that CMD_PREFETCH's
            // clear is the last assignment and wins for that cycle.
            prefetch_faults <= prefetch_faults_now;

            // -------------------------------------------------------
            // TOCTOU trap check: on address phase completion, compare
            // against all 4 trap entries. All matches trigger; the highest
            // already-triggered matching index selects the redirect.
            //
            // - Armed && !Triggered: mark triggered (serve original data)
            // - Armed && Triggered:  activate redirect (serve replacement)
            // -------------------------------------------------------
            log_addr_valid_prev <= log_addr_valid_sync;
            trap_check_pending <= log_addr_event;
            trap_notify_strobe <= 0;

            if (log_addr_event) begin
                // Address payload and match bits advance together. The
                // logger consumes the payload only with the later strobe.
                trap_notify_addr <= log_addr_sync;
                for (i = 0; i < 4; i = i + 1)
                    trap_match_pending[i] <= trap_armed[i] &&
                        ((log_addr_sync & trap_mask[i]) == trap_start[i]);
            end
            if (trap_check_pending) begin
                for (i = 0; i < 4; i = i + 1) begin
                    if (trap_match_pending[i]) begin
                        if (!trap_triggered[i]) begin
                            // First access: mark triggered, serve original data
                            trap_triggered[i] <= 1;
                        end
                        else begin
                            // Second+ access: activate redirect
                            redirect_active <= 1;
                            // The redirect is applied to the 23-bit burst
                            // address, so the byte-granular mask and
                            // replacement base enter it >>3. The low three
                            // byte bits are not expressible: an SDRAM burst
                            // is 8 bytes, so a match finer than that cannot
                            // be redirected to a different offset.
                            redirect_mask <= {2'b00, trap_mask[i][23:3]};
                            // Compute replacement burst addr via bitwise mux:
                            // new_burst = (replace & mask) | (original & ~mask)
                            redirect_base <= {2'b00, trap_replace[i][23:3]};
                            
                            trap_notify_strobe <= 1;
                            trap_notify_index  <= i[1:0];
                        end
                    end
                end
            end
            
            // Also suppress a pending redirect that completes after CS
            // deasserts; it must not leak into the following transaction.
            if (!spi_active_sync)
                redirect_active <= 0;
                
            // Serial protocol idle timeout: reset parser if stuck in
            // multi-byte command header with no data for ~137us.
            // Exclude active read/write operations: during reads the
            // host sends nothing (waiting for response data), so the
            // idle counter would fire and kill the transfer.
            if (in_count != 0 && !rxd_strobe_buf &&
                (read_state == 3'd0) && (write_state == 3'd0)) begin
                serial_idle_count <= serial_idle_count + 1;
                if (serial_idle_expired) begin
                    in_count <= 0;
                    cmd <= CMD_NOP;
                end
            end
            else begin
                serial_idle_count <= 0;
            end
            
            // -----------------------------------------------------------
            // Always-safe command dispatcher.
            //
            // Handles VERSION/START/STOP/STATUS/PREFETCH and starts
            // LOGPOLL regardless of SPI state.
            // None of these touch SDRAM, chip configuration, or spi_trx
            // state, so processing them while SPI emulation is live is
            // safe -- and is exactly what lets the host recover control
            // (via STOP) when a target is hammering the SPI bus.  The
            // parser must be idle (cmd_idle) so we do not collide with
            // an in-progress multi-byte command.
            // -----------------------------------------------------------
            if (rxd_strobe_buf && cmd_idle) begin
                case (rxd_data_buf)
                CMD_VERSION: begin
                    txd_strobe_buf <= 1;
                    txd_data_buf <= VERSION;
                end
                CMD_START: begin
                    spi_run_requested <= 1;
                    // The init completion path replies to an earlier START.
                    // Include this edge's final write to avoid losing an
                    // ACK when START coincides with initialization completion.
                    if (pp_init_done || pp_init_finishing) begin
                        txd_strobe_buf <= 1;
                        txd_data_buf <= 8'h01;
                    end
                end
                CMD_STOP: begin
                    spi_run_requested <= 0;
                    txd_strobe_buf <= 1;
                    txd_data_buf <= 8'h01;
                end
                CMD_STATUS: begin
                    txd_strobe_buf <= 1;
                    // 0x01 = running, 0x02 = stopped.  Using two non-zero
                    // codes (rather than 0x00 for stopped) lets the host
                    // tool's transparent 0x00 skipping still work around
                    // the FT2232H's occasional leaked modem-status bytes.
                    txd_data_buf <= spi_running ? 8'h01 : 8'h02;
                end
                CMD_PREFETCH: begin
                    // Reports and clears the latched flags, so a host can
                    // tell whether the fast read path ever came up short
                    // while the target was reading. Safe to issue at any
                    // time: it touches neither SDRAM nor spi_trx.
                    txd_strobe_buf <= 1;
                    // read_ack skips 0x00 and 0xFF as transport noise;
                    // include a marker even when neither fault is set.
                    txd_data_buf <= PREFETCH_VALID | {6'd0, prefetch_faults_now};
                    prefetch_faults <= 2'b00;
                end
                CMD_LOGPOLL: begin
                    // LOGPOLL: no argument bytes.  Enter drain state
                    // machine which emits up to LOG_POLL_MAX bytes from
                    // the logger FIFO followed by a 0xA0 terminator.
                    // The state machine runs outside the SPI-gated
                    // dispatcher so polls complete even while a flash
                    // read is in progress on the SPI pins.
                    cmd             <= CMD_LOGPOLL;
                    log_poll_state  <= 2'd1;
                    log_poll_remain <= LOG_POLL_MAX;
                end
                default: ;  // fall through to gated dispatcher below
                endcase
            end

            // -----------------------------------------------------------
            // CMD_LOGPOLL drain state machine (always runs, ungated).
            //
            // LOGPOLL only touches the logger FIFO and the host TX path,
            // so it is safe to process even while the SPI master is
            // actively hammering the bus.  Running ungated also means
            // monitor tools can poll while logging real-time traffic.
            //
            //   State 1: pop one raw byte per TX slot while FIFO has data
            //            and the per-poll cap has not been exhausted.  If
            //            the raw byte collides with framing, send ESCAPE
            //            now and state 3 sends the escape code next.
            //   State 2: emit 0xA0 terminator and return to idle.
            //   State 3: emit second byte of an escaped raw log byte.
            // -----------------------------------------------------------
            if (log_poll_state == 2'd1 && can_send) begin
                if (log_fifo_data_available && log_poll_remain != 0) begin
                    txd_strobe_buf       <= 1;
                    log_fifo_read_strobe <= 1;
                    log_poll_remain      <= log_poll_remain - 1;
                    if (log_fifo_read_data == LOG_POLL_TERMINATOR) begin
                        txd_data_buf         <= LOG_POLL_ESCAPE;
                        log_poll_escape_code <= 8'h00;
                        log_poll_state       <= 2'd3;
                    end
                    else if (log_fifo_read_data == LOG_POLL_ESCAPE) begin
                        txd_data_buf         <= LOG_POLL_ESCAPE;
                        log_poll_escape_code <= 8'h05;
                        log_poll_state       <= 2'd3;
                    end
                    else begin
                        txd_data_buf <= log_fifo_read_data;
                    end
                end
                else begin
                    log_poll_state <= 2'd2;
                end
            end
            else if (log_poll_state == 2'd2 && can_send) begin
                txd_strobe_buf <= 1;
                txd_data_buf   <= LOG_POLL_TERMINATOR;
                log_poll_state <= 2'd0;
                cmd            <= CMD_NOP;
            end
            else if (log_poll_state == 2'd3 && can_send) begin
                txd_strobe_buf <= 1;
                txd_data_buf   <= log_poll_escape_code;
                log_poll_state <= 2'd1;
            end

            // -----------------------------------------------------------
            // Command parser and SDRAM state machines.
            //
            // Popped bytes were already checked against the SDRAM gate
            // (ft_rx_take/uart_rx_take), so every popped byte is parsed.
            // The serial SDRAM read/write state machines themselves run
            // only while the gate is open, so they never race an in-flight
            // SPI transaction or the program engine. RAMREAD, RAMWRITE and
            // CHIPCONFIG are additionally only accepted while emulation is
            // stopped, so the host cannot disturb live emulation.
            // -----------------------------------------------------------
            begin
                if (rxd_strobe_buf) begin
                    serial_idle_count <= 0;

                    if (in_count == 0) begin
                        // The always-safe opcodes are handled above.
                        // HOLDCTL/LOGCTL/TOCTOU only flip flags/GPIO and
                        // do not touch SDRAM or spi_trx state, so they
                        // are safe to accept in either spi_running state.
                        // Destructive commands (RAMREAD/RAMWRITE/CHIPCONFIG)
                        // are only accepted when SPI emulation is stopped
                        // so they cannot contend for SDRAM or cfg registers.
                        if (rxd_data_buf == CMD_HOLDCTL) begin
                            cmd <= CMD_HOLDCTL;
                            in_count <= 1;
                        end
                        else if (rxd_data_buf == CMD_LOGCTL) begin
                            cmd <= CMD_LOGCTL;
                            in_count <= 1;
                        end
                        else if (rxd_data_buf == CMD_TOCTOU) begin
                            cmd <= CMD_TOCTOU;
                            in_count <= 1;
                        end
                        else if (!spi_run_requested) begin
                            if (rxd_data_buf == CMD_RAMREAD ||
                                rxd_data_buf == CMD_RAMWRITE) begin
                                cmd <= rxd_data_buf;
                                in_count <= 1;

                                read_state <= 0;
                                read_pos <= 0;

                                write_state <= 0;
                                write_pos <= 0;
                            end
                            else if (rxd_data_buf == CMD_CHIPCONFIG) begin
                                cmd <= CMD_CHIPCONFIG;
                                in_count <= 1;
                            end
                        end
                    end
                    else if (cmd == CMD_CHIPCONFIG) begin
                        // -----------------------------------------------
                        // CHIPCONFIG protocol:
                        //   Byte 1:   JEDEC manufacturer ID
                        //   Byte 2:   JEDEC device ID high
                        //   Byte 3:   JEDEC device ID low
                        //   Byte 4:   Flags (bit 0 = 4-byte addr support)
                        //   Byte 5-7: Chip erase burst count (23-bit MSB)
                        //   Byte 8:   SFDP table length (0-128)
                        //   Byte 9+:  SFDP table data
                        // Response: 0x01
                        // -----------------------------------------------
                        
                        // Store header bytes into config registers.
                        // JEDEC ID is packed as {device_lo, device_hi, manufacturer}
                        // so that READID outputs [7:0]=manufacturer first, matching
                        // the SPI RDID response order (manufacturer, type, capacity).
                        if (in_count == 8'd1)
                            cfg_jedec_id[7:0] <= rxd_data_buf;   // manufacturer
                        else if (in_count == 8'd2)
                            cfg_jedec_id[15:8] <= rxd_data_buf;  // device high (type)
                        else if (in_count == 8'd3)
                            cfg_jedec_id[23:16] <= rxd_data_buf; // device low (capacity)
                        else if (in_count == 8'd4)
                            cfg_4byte <= rxd_data_buf[0];
                        else if (in_count == 8'd5)
                            cfg_chip_erase_bursts[22:16] <= rxd_data_buf[6:0];
                        else if (in_count == 8'd6)
                            cfg_chip_erase_bursts[15:8] <= rxd_data_buf;
                        else if (in_count == 8'd7)
                            cfg_chip_erase_bursts[7:0] <= rxd_data_buf;
                        else if (in_count == 8'd8) begin
                            cfg_sfdp_remaining <= rxd_data_buf;
                            sfdp_valid_len <= 0;
                            sfdp_wr_pos <= 0;
                        end
                        // SFDP data bytes: single-cycle BSRAM write pulse.
                        else begin
                            sfdp_waddr <= sfdp_wr_pos;
                            sfdp_wdata <= rxd_data_buf;
                            sfdp_wren <= 1'b1;
                            // Only received bytes are valid, even if an
                            // incomplete CHIPCONFIG later times out.
                            if (sfdp_valid_len < 8'd128)
                                sfdp_valid_len <= sfdp_valid_len + 1'b1;
                            sfdp_wr_pos <= sfdp_wr_pos + 1;
                            cfg_sfdp_remaining <= cfg_sfdp_remaining - 1;
                        end
                        
                        // Advance or finish
                        if (in_count == 8'd8 && rxd_data_buf == 0) begin
                            // No SFDP data, done immediately
                            txd_strobe_buf <= 1;
                            txd_data_buf <= 8'h01;
                            in_count <= 0;
                            cmd <= CMD_NOP;
                        end
                        else if (in_count > 8'd8 && cfg_sfdp_remaining == 8'd1) begin
                            // Last SFDP byte received, done
                            txd_strobe_buf <= 1;
                            txd_data_buf <= 8'h01;
                            in_count <= 0;
                            cmd <= CMD_NOP;
                        end
                        else begin
                            in_count <= in_count + 1;
                        end
                    end
                    else if (cmd == CMD_HOLDCTL) begin
                        // -----------------------------------------------
                        // HOLDCTL protocol:
                        //   Byte 1: 0x01 = assert #HOLD (silence target flash)
                        //           0x00 = release #HOLD (target flash active)
                        // Response: 0x01
                        //
                        // Mutually exclusive with quad I/O: when hold is
                        // asserted, IO3 is driven low continuously to keep
                        // the target flash in hold state.
                        // -----------------------------------------------
                        hold_out <= rxd_data_buf[0];
                        txd_strobe_buf <= 1;
                        txd_data_buf <= 8'h01;
                        in_count <= 0;
                        cmd <= CMD_NOP;
                    end
                    else if (cmd == CMD_LOGCTL) begin
                        // -----------------------------------------------
                        // LOGCTL protocol:
                        //   Byte 1: 0x01 = start logging, 0x00 = stop
                        // Response: 0x01
                        // -----------------------------------------------
                        log_active <= rxd_data_buf[0];
                        txd_strobe_buf <= 1;
                        txd_data_buf <= 8'h01;
                        in_count <= 0;
                        cmd <= CMD_NOP;
                    end
                    else if (cmd == CMD_TOCTOU) begin
                        // -----------------------------------------------
                        // TOCTOU protocol:
                        //   Byte 1: sub-command
                        //     0x01 SET:  +index(1) +start(3) +mask(3) +replace(3)
                        //     0x02 ARM:  +index(1)
                        //     0x03 DISARM: +index(1)
                        //     0x04 RESET:  +index(1)
                        //     0x05 RESET_ALL: (no extra bytes)
                        // Response: 0x01 on completion
                        // -----------------------------------------------
                        if (in_count == 1) begin
                            toctou_sub_cmd <= rxd_data_buf;
                            if (rxd_data_buf == TOCTOU_RESET_ALL) begin
                                // Cancel comparisons captured on this edge too.
                                trap_match_pending <= 0;
                                trap_armed <= 0;
                                trap_triggered <= 0;
                                redirect_active <= 0;
                                txd_strobe_buf <= 1;
                                txd_data_buf <= 8'h01;
                                in_count <= 0;
                                cmd <= CMD_NOP;
                            end
                            else
                                in_count <= 2;
                        end
                        else if (in_count == 2) begin
                            toctou_index <= rxd_data_buf[1:0];
                            if (toctou_sub_cmd == TOCTOU_ARM) begin
                                trap_armed[rxd_data_buf[1:0]] <= 1;
                                txd_strobe_buf <= 1;
                                txd_data_buf <= 8'h01;
                                in_count <= 0;
                                cmd <= CMD_NOP;
                            end
                            else if (toctou_sub_cmd == TOCTOU_DISARM) begin
                                trap_match_pending[rxd_data_buf[1:0]] <= 0;
                                trap_armed[rxd_data_buf[1:0]] <= 0;
                                txd_strobe_buf <= 1;
                                txd_data_buf <= 8'h01;
                                in_count <= 0;
                                cmd <= CMD_NOP;
                            end
                            else if (toctou_sub_cmd == TOCTOU_RESET) begin
                                trap_match_pending[rxd_data_buf[1:0]] <= 0;
                                trap_triggered[rxd_data_buf[1:0]] <= 0;
                                txd_strobe_buf <= 1;
                                txd_data_buf <= 8'h01;
                                in_count <= 0;
                                cmd <= CMD_NOP;
                            end
                            else
                                in_count <= 3;
                        end
                        // TOCTOU SET: receive start(3), mask(3), replace(3)
                        else if (in_count == 3)  begin toctou_start_buf[23:16] <= rxd_data_buf; in_count <= 4; end
                        else if (in_count == 4)  begin toctou_start_buf[15:8]  <= rxd_data_buf; in_count <= 5; end
                        else if (in_count == 5)  begin toctou_start_buf[7:0]   <= rxd_data_buf; in_count <= 6; end
                        else if (in_count == 6)  begin toctou_mask_buf[23:16]  <= rxd_data_buf; in_count <= 7; end
                        else if (in_count == 7)  begin toctou_mask_buf[15:8]   <= rxd_data_buf; in_count <= 8; end
                        else if (in_count == 8)  begin toctou_mask_buf[7:0]    <= rxd_data_buf; in_count <= 9; end
                        else if (in_count == 9)  begin
                            trap_start[toctou_index] <= toctou_start_buf & toctou_mask_buf;
                            trap_mask[toctou_index]  <= toctou_mask_buf;
                            trap_replace[toctou_index][23:16] <= rxd_data_buf;
                            in_count <= 10;
                        end
                        else if (in_count == 10) begin trap_replace[toctou_index][15:8] <= rxd_data_buf; in_count <= 11; end
                        else if (in_count == 11) begin
                            trap_replace[toctou_index][7:0] <= rxd_data_buf;
                            trap_match_pending[toctou_index] <= 0;
                            trap_triggered[toctou_index] <= 0;
                            txd_strobe_buf <= 1;
                            txd_data_buf <= 8'h01;
                            in_count <= 0;
                            cmd <= CMD_NOP;
                        end
                    end
                    else begin
                        // RAMREAD / RAMWRITE handling (v3: 6-byte header)
                        // Header: cmd, addr[2], addr[1], addr[0], len_hi, len_lo
                        if (in_count <= 3)
                            addr <= {addr[14:0], rxd_data_buf};
                        else if (in_count == 4)
                            len[15:8] <= rxd_data_buf;
                        else if (in_count == 5)
                            len[7:0] <= rxd_data_buf;

                        if (cmd == CMD_RAMREAD && in_count == 5) begin
                            read_state <= 1;
                        end
                        if (cmd == CMD_RAMWRITE && in_count > 5) begin
                            write_buffer[write_pos*8 +: 8] <= rxd_data_buf;
                            
                            if (write_pos == 7) begin
                                write_strobe <= 1;
                            end
                            write_pos <= write_pos + 1;
                        end

                        if (in_count <= 5)
                            in_count <= in_count + 1;
                    end

                end
                else if (serial_gate) begin

                    if (write_strobe && !sdram_busy)
                        write_state <= 1;

                    if (read_state != 3'd0) begin
                        if ((read_state == 1) && !sdram_busy) begin
                            // Activate
                            host_access_cmd <= 2'b11;
                            read_state <= 2;
                        end
                        else if ((read_state == 2) && !sdram_busy) begin
                            // Read
                            host_access_cmd <= 2'b01;
                            read_state <= 3;
                        end
                        else if ((read_state == 3) && !sdram_busy && can_send) begin
                            txd_strobe_buf <= 1;
                            txd_data_buf <= sdram_read_buffer[read_pos*8 +: 8];

                            if (read_pos == 7) begin
                                if (len == 1) begin
                                    read_state <= 0;
                                    in_count <= 0;
                                    cmd <= CMD_NOP;
                                end
                                else begin
                                    addr <= addr + 1;
                                    len <= len - 1;
                                    read_state <= 1;
                                    read_pos <= 0;
                                end
                            end
                            else
                                read_pos <= read_pos + 1;
                        end
                    end
                    else if (write_state != 3'd0) begin
                        if ((write_state == 1) && !sdram_busy) begin
                            // Activate
                            host_access_cmd <= 2'b11;
                            write_strobe <= 0;
                            write_state <= 2;
                        end
                        else if ((write_state == 2) && !sdram_busy) begin
                            // Write
                            host_access_cmd <= 2'b10;
                            write_state <= 3;
                        end
                        else if ((write_state == 3) && !sdram_busy) begin
                            if (len == 1) begin
                                if (can_send) begin
                                    txd_strobe_buf <= 1;
                                    txd_data_buf <= 8'h01;
                                    write_state <= 0;
                                    in_count <= 0;
                                    cmd <= CMD_NOP;
                                end
                            end
                            else begin
                                write_state <= 0;
                                addr <= addr + 1;
                                len <= len - 1;
                            end
                        end
                    end
                end
            end
        end
    end

endmodule
