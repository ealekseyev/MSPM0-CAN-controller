#ifndef MCP2515_BITRATES_H
#define MCP2515_BITRATES_H

typedef enum {
    MCP_8MHZ,
    MCP_16MHZ
} mcp2515_clk_t;

typedef enum {
    CAN_5KBPS,
    CAN_10KBPS,
    CAN_20KBPS,
    CAN_31K25BPS,
    CAN_33KBPS,
    CAN_40KBPS,
    CAN_50KBPS,
    CAN_80KBPS,
    CAN_83K3BPS,
    CAN_95KBPS,
    CAN_100KBPS,
    CAN_125KBPS,
    CAN_200KBPS,
    CAN_250KBPS,
    CAN_500KBPS,
    CAN_1000KBPS
} mcp2515_speed_t;


typedef struct {
    uint8_t cnf1;
    uint8_t cnf2;
    uint8_t cnf3;
} mcp2515_timing_t;

/*
 *  Speed 8MHz
 */
#define MCP_8MHz_1000kBPS  ((mcp2515_timing_t){0x00, 0x80, 0x80})
#define MCP_8MHz_500kBPS   ((mcp2515_timing_t){0x00, 0x90, 0x82})
#define MCP_8MHz_250kBPS   ((mcp2515_timing_t){0x00, 0xB1, 0x85})
#define MCP_8MHz_200kBPS   ((mcp2515_timing_t){0x00, 0xB4, 0x86})
#define MCP_8MHz_125kBPS   ((mcp2515_timing_t){0x01, 0xB1, 0x85})
#define MCP_8MHz_100kBPS   ((mcp2515_timing_t){0x01, 0xB4, 0x86})
#define MCP_8MHz_80kBPS    ((mcp2515_timing_t){0x01, 0xBF, 0x87})
#define MCP_8MHz_50kBPS    ((mcp2515_timing_t){0x03, 0xB4, 0x86})
#define MCP_8MHz_40kBPS    ((mcp2515_timing_t){0x03, 0xBF, 0x87})
#define MCP_8MHz_33k3BPS   ((mcp2515_timing_t){0x47, 0xE2, 0x85})
#define MCP_8MHz_31k25BPS  ((mcp2515_timing_t){0x07, 0xA4, 0x84})
#define MCP_8MHz_20kBPS    ((mcp2515_timing_t){0x07, 0xBF, 0x87})
#define MCP_8MHz_10kBPS    ((mcp2515_timing_t){0x0F, 0xBF, 0x87})
#define MCP_8MHz_5kBPS     ((mcp2515_timing_t){0x1F, 0xBF, 0x87})

/*
 *  Speed 16MHz
 */
#define MCP_16MHz_1000kBPS ((mcp2515_timing_t){0x00, 0xD0, 0x82})
#define MCP_16MHz_500kBPS  ((mcp2515_timing_t){0x00, 0xF0, 0x86})
#define MCP_16MHz_250kBPS  ((mcp2515_timing_t){0x41, 0xF1, 0x85})
#define MCP_16MHz_200kBPS  ((mcp2515_timing_t){0x01, 0xFA, 0x87})
#define MCP_16MHz_125kBPS  ((mcp2515_timing_t){0x03, 0xF0, 0x86})
#define MCP_16MHz_100kBPS  ((mcp2515_timing_t){0x03, 0xFA, 0x87})
#define MCP_16MHz_95kBPS   ((mcp2515_timing_t){0x03, 0xAD, 0x07})
#define MCP_16MHz_83k3BPS  ((mcp2515_timing_t){0x03, 0xBE, 0x07})
#define MCP_16MHz_80kBPS   ((mcp2515_timing_t){0x03, 0xFF, 0x87})
#define MCP_16MHz_50kBPS   ((mcp2515_timing_t){0x07, 0xFA, 0x87})
#define MCP_16MHz_40kBPS   ((mcp2515_timing_t){0x07, 0xFF, 0x87})
#define MCP_16MHz_33k3BPS  ((mcp2515_timing_t){0x4E, 0xF1, 0x85})
#define MCP_16MHz_20kBPS   ((mcp2515_timing_t){0x0F, 0xFF, 0x87})
#define MCP_16MHz_10kBPS   ((mcp2515_timing_t){0x1F, 0xFF, 0x87})
#define MCP_16MHz_5kBPS    ((mcp2515_timing_t){0x3F, 0xFF, 0x87})

#endif
