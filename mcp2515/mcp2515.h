#ifndef MCP2515_DRIVER_H
#define MCP2515_DRIVER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "ti/driverlib/driverlib.h"
#include "ti/driverlib/m0p/dl_core.h"   // delay_cycles, POWER_STARTUP_DELAY
#include "ti/driverlib/dl_common.h"
#include "mcp2515_registers.h"
#include "mcp2515_bitrates.h"
#include "mcp2515_errors.h"

#ifndef MCP2515_RING_BUFFER_SIZE
#define MCP2515_RING_BUFFER_SIZE 50
#endif

typedef struct {
    uint8_t reg;
    uint8_t payload[14];
    size_t data_len;
} mcp2515_payload_t;

typedef enum {
    MCP2515_RX_NONE = 0,
    MCP2515_RX_BUF0 = 1,
    MCP2515_RX_BUF1 = 2,
} mcp2515_rx_status_t;

typedef enum {
    MCP2515_TX_OK        = 0,
    MCP2515_TX_FAIL      = 1,  // dlc too large, or post-RTS error flag set
    MCP2515_TX_ALL_BUSY  = 2,  // TXB0/1/2 all have TXREQ pending
} mcp2515_tx_status_t;

typedef struct {
    uint32_t timestamp; // capture time (user-defined units)
    uint32_t id;      // 11-bit standard CAN ID, or 29-bit extended CAN ID
    uint8_t  ext;     // 1 = extended frame (29-bit ID), 0 = standard
    uint8_t  rtr;     // 1 = remote frame
    uint8_t  dlc;     // data length code (0–8)
    uint8_t  data[8]; // data bytes
} mcp2515_frame_t;

typedef struct {
    mcp2515_frame_t buffer[MCP2515_RING_BUFFER_SIZE];
    size_t head;   // write index
    size_t tail;   // read index
    size_t count;  // number of items currently in buffer
} mcp2515_ring_t;


void _spi_write(uint8_t *buf, size_t len);
void _spi_read(uint8_t *tx_buf, size_t tx_len, uint8_t *rx_buf, size_t rx_len);
uint8_t mcp2515_read_register(uint8_t addr);
void mcp2515_write_register(uint8_t addr, uint8_t val);
void mcp2515_modify_register(uint8_t addr, uint8_t mask, uint8_t val);
void mcp2515_reset();
void mcp2515_set_bitrate(mcp2515_speed_t can_speed, mcp2515_clk_t crystal_mhz);
uint8_t mcp2515_init(SPI_Regs *spi);
mcp2515_tx_status_t mcp2515_write_frame(const mcp2515_frame_t *frame);
int mcp2515_read(mcp2515_frame_t *frame);
mcp2515_rx_status_t mcp2515_rxbuf_status(void);
size_t              mcp2515_available(void);
mcp2515_frame_t     mcp2515_read_can(void);
mcp2515_frame_t mcp2515_read_frame(mcp2515_rx_status_t rxbuf);
void mcp2515_service_rx(void);

/* Ring buffer helpers */
void   mcp2515_ring_init(mcp2515_ring_t *r);
bool   mcp2515_ring_push(mcp2515_ring_t *r, const mcp2515_frame_t *frame);
mcp2515_frame_t *mcp2515_ring_reserve(mcp2515_ring_t *r);
size_t mcp2515_ring_size(const mcp2515_ring_t *r);
bool   mcp2515_ring_advance(mcp2515_ring_t *r);
mcp2515_frame_t *mcp2515_ring_peek(mcp2515_ring_t *r);
bool   mcp2515_ring_empty(const mcp2515_ring_t *r);
bool   mcp2515_ring_full(const mcp2515_ring_t *r);

extern void GROUP1_IRQHandler(void);
#endif