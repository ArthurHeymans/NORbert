// Host serial protocol shared by the UART and FT245 transports: command
// opcodes, TOCTOU sub-commands and the CMD_LOGPOLL framing and packet
// types. Included inside glue.v and logger.v.
//
// tool/src/rtl_sync.rs parses this file and checks tool/src/protocol.rs
// against it, so `cargo test` fails if the two drift apart.

localparam VERSION = 8'h05;  // Version 5: HOLDCTL + logging + TOCTOU

localparam
    CMD_NOP          = 8'h00,
    CMD_VERSION      = 8'h30,
    CMD_RAMREAD      = 8'h31,
    CMD_RAMWRITE     = 8'h32,
    CMD_CHIPCONFIG   = 8'h33,
    CMD_START        = 8'h34,  // Enable SPI emulation
    CMD_STOP         = 8'h35,  // Disable SPI emulation
    CMD_STATUS       = 8'h36,  // Query running state
    CMD_HOLDCTL      = 8'h37,  // Assert/release target flash #HOLD
    CMD_LOGCTL       = 8'h38,  // Enable/disable SPI bus logging capture
    CMD_TOCTOU       = 8'h39,  // TOCTOU trap management
    CMD_LOGPOLL      = 8'h3A;  // Drain logger ring FIFO

// CMD_TOCTOU sub-commands
localparam
    TOCTOU_SET       = 8'h01,
    TOCTOU_ARM       = 8'h02,
    TOCTOU_DISARM    = 8'h03,
    TOCTOU_RESET     = 8'h04,
    TOCTOU_RESET_ALL = 8'h05;

// Terminator byte appended to every CMD_LOGPOLL response.  Log data
// bytes equal to the terminator or escape byte are byte-stuffed as
// 0xA5 0x00 (for 0xA0) or 0xA5 0x05 (for 0xA5), so the terminator is
// unambiguous even when raw packet payloads contain 0xA0.
localparam LOG_POLL_TERMINATOR = 8'hA0;
localparam LOG_POLL_ESCAPE     = 8'hA5;

// Log packet types written by logger.v (payload lengths in bytes)
localparam
    LOG_PKT_CMD      = 8'hA1,  // opcode (1)
    LOG_PKT_ADDR     = 8'hA2,  // address (4)
    LOG_PKT_END      = 8'hA3,  // byte count (3)
    LOG_PKT_TRAP     = 8'hA4;  // trap index, address (3), pad (1)
