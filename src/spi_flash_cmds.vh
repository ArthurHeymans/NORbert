// SPI flash opcodes decoded by spi_trx.v, and the read wait states it
// implements. Included inside spi_trx.
//
// tool/src/rtl_sync.rs parses this file: the host tool's opcode names and
// the SFDP table it generates must agree with it, or `cargo test` fails.

localparam
    CMD_WRITESTATUS     = 8'h01,  // Write Status Register (1-2 data bytes)
    CMD_PAGEPROGRAM     = 8'h02,
    CMD_READ            = 8'h03,
    CMD_WRITEDISABLE    = 8'h04,
    CMD_READSTATUS      = 8'h05,
    CMD_WRITEENABLE     = 8'h06,
    CMD_FASTREAD        = 8'h0B,
    CMD_FASTREAD_4B     = 8'h0C,
    CMD_PAGEPROGRAM_4B  = 8'h12,
    CMD_READ_4B         = 8'h13,
    CMD_READSTATUS3     = 8'h15,  // Or legacy Atmel ID, selected by profile
    CMD_RDBANK         = 8'h16,
    CMD_WRBANK         = 8'h17,
    CMD_WRITESTATUS2   = 8'h31,
    CMD_SECTORPROTECT  = 8'h36,
    CMD_SECTORUNPROTECT = 8'h39,
    CMD_WBPR           = 8'h42,
    CMD_RDUID          = 8'h4B,
    CMD_CHIPERASE_AT25F = 8'h62,
    CMD_RESETENABLE    = 8'h66,
    CMD_RESET          = 8'h99,
    CMD_RBPR           = 8'h72,
    CMD_ULBPR          = 8'h98,
    CMD_WREAR          = 8'hC5,
    CMD_RDEAR          = 8'hC8,
    CMD_ERASE_256B     = 8'h81,
    CMD_ERASE_D7       = 8'hD7,
    CMD_ERASE_DB       = 8'hDB,
    CMD_DUALREAD_4B     = 8'h3C,  // Or Atmel Read Sector Protection Register
    CMD_SECTORERASE_4K  = 8'h20,
    CMD_SECTORERASE_4K_4B = 8'h21,
    CMD_READSTATUS2     = 8'h35,  // Read Status Register 2
    CMD_DUALREAD        = 8'h3B,  // Dual Output Read (1-1-2)
    CMD_BLOCKERASE_32K  = 8'h52,
    CMD_BLOCKERASE_32K_4B = 8'h5C,
    CMD_CHIPERASE1      = 8'h60,
    CMD_QUADREAD        = 8'h6B,  // Quad Output Read (1-1-4)
    CMD_QUADREAD_4B     = 8'h6C,  // Quad Output Read with 4-byte address
    CMD_READID1         = 8'h9E,
    CMD_READSFDP        = 8'h5A,  // Read SFDP table
    CMD_READID2         = 8'h9F,
    CMD_4BYTEENABLE     = 8'hB7,
    CMD_DUALIOREAD      = 8'hBB,  // Dual I/O Read (1-2-2)
    CMD_DUALIOREAD_4B   = 8'hBC,  // Dual I/O Read with 4-byte address
    CMD_CHIPERASE2      = 8'hC7,
    CMD_BLOCKERASE_64K  = 8'hD8,
    CMD_BLOCKERASE_64K_4B = 8'hDC,
    CMD_EWSR            = 8'h50,  // Or AT26DF041 2KiB erase, selected by profile
    CMD_AAI_WORD        = 8'hAD,  // AAI Word Program (SST)
    CMD_4BYTEDISABLE    = 8'hE9,
    CMD_QUADIOREAD      = 8'hEB,  // Quad I/O Read (1-4-4)
    CMD_QUADIOREAD_4B   = 8'hEC;  // Quad I/O Read with 4-byte address

// 0xF2 is not a NOR flash command. It used to be an internal logging hook
// with no consumer left; unlisted opcodes are ignored by the decoder.

// Wait states between the address and the first data clock of fast reads,
// in SPI clocks. spi_prefetch's lookahead post points (dummy_count 5, mode
// clocks 3/1 left) assume these values.
localparam FAST_READ_DUMMY_CLKS = 8;  // 0x0B, 0x3B, 0x6B, 0x5A (and 4-byte forms)
localparam DUAL_IO_MODE_CLKS    = 4;  // 0xBB: mode byte on IO1:IO0, no dummy
localparam QUAD_IO_MODE_CLKS    = 2;  // 0xEB: mode byte on IO3:IO0...
localparam QUAD_IO_DUMMY_CLKS   = 4;  // ...followed by dummy clocks
