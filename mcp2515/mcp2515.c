#include "mcp2515.h"
#include "ti_msp_dl_config.h"
#include "ti/driverlib/driverlib.h"
#include "ti/driverlib/m0p/dl_core.h"   // delay_cycles, POWER_STARTUP_DELAY
#include "ti/driverlib/dl_common.h"
#include <string.h>

static SPI_Regs *spi_inst;
static mcp2515_timing_t bitrate_prescaler;
static mcp2515_ring_t msgbuf;


/*              RING BUFFER HELPERS              */

void mcp2515_ring_init(mcp2515_ring_t *r) {
    r->head = 0;
    r->tail = 0;
    r->count = 0;
}

// push a frame at head. returns false if full (frame dropped).
bool mcp2515_ring_push(mcp2515_ring_t *r, const mcp2515_frame_t *frame) {
    if (r->count >= MCP2515_RING_BUFFER_SIZE) return false;
    r->buffer[r->head] = *frame;
    r->head = (r->head + 1) % MCP2515_RING_BUFFER_SIZE;
    r->count++;
    return true;
}

// reserve the next head slot: advances head + count and returns a
// pointer to the slot for the caller to write into directly.
// returns NULL if the buffer is full.
mcp2515_frame_t *mcp2515_ring_reserve(mcp2515_ring_t *r) {
    if (r->count >= MCP2515_RING_BUFFER_SIZE) return NULL;
    mcp2515_frame_t *slot = &r->buffer[r->head];
    r->head = (r->head + 1) % MCP2515_RING_BUFFER_SIZE;
    r->count++;
    return slot;
}

// current number of items in the buffer
size_t mcp2515_ring_size(const mcp2515_ring_t *r) {
    return r->count;
}

// advance (pop) the tail. returns false if empty.
bool mcp2515_ring_advance(mcp2515_ring_t *r) {
    if (r->count == 0) return false;
    r->tail = (r->tail + 1) % MCP2515_RING_BUFFER_SIZE;
    r->count--;
    return true;
}

// get pointer to the oldest frame (at tail), or NULL if empty.
mcp2515_frame_t *mcp2515_ring_peek(mcp2515_ring_t *r) {
    if (r->count == 0) return NULL;
    return &r->buffer[r->tail];
}

bool mcp2515_ring_empty(const mcp2515_ring_t *r) {
    return r->count == 0;
}

bool mcp2515_ring_full(const mcp2515_ring_t *r) {
    return r->count >= MCP2515_RING_BUFFER_SIZE;
}

/*           SPI READ/WRITE FUNCTIONS            */
void _spi_write(uint8_t *buf, size_t len) {
    DL_GPIO_clearPins(GPIOA, SPI_PINS_CS_PIN);

    for (size_t i = 0; i < len; i++) {
        while (!DL_SPI_isTXFIFOEmpty(spi_inst));
        DL_SPI_transmitData8(spi_inst, buf[i]);
        DL_SPI_receiveDataBlocking8(spi_inst);
    }
    while (DL_SPI_isBusy(spi_inst));

    DL_GPIO_setPins(GPIOA, SPI_PINS_CS_PIN);
}

void _spi_read(uint8_t *tx_buf, size_t tx_len, uint8_t *rx_buf, size_t rx_len) {
    DL_GPIO_clearPins(GPIOA, SPI_PINS_CS_PIN);

    // transmit phase (command + address bytes), discard RX
    for (size_t i = 0; i < tx_len; i++) {
        while (!DL_SPI_isTXFIFOEmpty(spi_inst));
        DL_SPI_transmitData8(spi_inst, tx_buf[i]);
        DL_SPI_receiveDataBlocking8(spi_inst); // discard
    }

    // receive phase, clock out dummy bytes
    for (size_t i = 0; i < rx_len; i++) {
        while (!DL_SPI_isTXFIFOEmpty(spi_inst));
        DL_SPI_transmitData8(spi_inst, 0xFF); // dummy
        rx_buf[i] = DL_SPI_receiveDataBlocking8(spi_inst);
    }

    while (DL_SPI_isBusy(spi_inst));
    DL_GPIO_setPins(GPIOA, SPI_PINS_CS_PIN);
}

/* indexes into the 5-byte SIDH..DLC header block */
#define MCP_SIDH        0
#define MCP_SIDL        1
#define MCP_EID8        2
#define MCP_EID0        3
#define MCP_DLC         4

#define TXB_EXIDE_MASK  0x08  // SIDL bit 3: extended-ID flag
#define DLC_MASK        0x0F  // DLC register: low 4 bits
#define CAN_MAX_DLEN    8

/*          MCP REGISTER READ/WRITE FUNCTIONS           */
uint8_t mcp2515_read_register(uint8_t addr) {
    uint8_t buf[] = {MCP2515_SPI_READ, addr};
    uint8_t rxbuf[1];
    _spi_read(buf, 2, rxbuf, 1);
    return rxbuf[0];
}

void mcp2515_write_register(uint8_t addr, uint8_t val) {
    uint8_t buf[] = {MCP2515_SPI_WRITE, addr, val};
    _spi_write(buf, 3);
}

// MCP2515 BIT MODIFY: only bits in `mask` are affected; others are preserved.
void mcp2515_modify_register(uint8_t addr, uint8_t mask, uint8_t val) {
    uint8_t buf[] = {MCP2515_SPI_BITMOD, addr, mask, val};
    _spi_write(buf, 4);
}

// Read `len` sequential registers starting at `addr`.
static void mcp2515_read_registers(uint8_t addr, uint8_t *out, size_t len) {
    uint8_t cmd[] = {MCP2515_SPI_READ, addr};
    _spi_read(cmd, 2, out, len);
}

// Write `len` sequential registers starting at `addr` in a single CS-low burst.
// Composes [WRITE, addr, in[0..len-1]] into a stack buffer and emits via _spi_write.
static void mcp2515_write_registers(uint8_t addr, const uint8_t *in, size_t len) {
    uint8_t buf[2 + 5 + CAN_MAX_DLEN]; /* hdr(2) + sidh..dlc(5) + data(<=8) */
    buf[0] = MCP2515_SPI_WRITE;
    buf[1] = addr;
    memcpy(&buf[2], in, len);
    _spi_write(buf, 2 + len);
}

/*              HIGH LEVEL FUNCTIONS            */

void mcp2515_reset() {
    uint8_t reset_cmd = MCP2515_SPI_RESET;
    _spi_write(&reset_cmd, 1);
    delay_cycles(3200000);
}

void mcp2515_set_bitrate(mcp2515_speed_t can_speed, mcp2515_clk_t crystal_mhz) {
    switch (crystal_mhz) {
        case MCP_8MHZ:
            switch (can_speed) {
                case CAN_1000KBPS: bitrate_prescaler = MCP_8MHz_1000kBPS; break;
                case CAN_500KBPS:  bitrate_prescaler = MCP_8MHz_500kBPS;  break;
                case CAN_250KBPS:  bitrate_prescaler = MCP_8MHz_250kBPS;  break;
                case CAN_200KBPS:  bitrate_prescaler = MCP_8MHz_200kBPS;  break;
                case CAN_125KBPS:  bitrate_prescaler = MCP_8MHz_125kBPS;  break;
                case CAN_100KBPS:  bitrate_prescaler = MCP_8MHz_100kBPS;  break;
                case CAN_80KBPS:   bitrate_prescaler = MCP_8MHz_80kBPS;   break;
                case CAN_50KBPS:   bitrate_prescaler = MCP_8MHz_50kBPS;   break;
                case CAN_40KBPS:   bitrate_prescaler = MCP_8MHz_40kBPS;   break;
                case CAN_33KBPS:   bitrate_prescaler = MCP_8MHz_33k3BPS;  break;
                case CAN_31K25BPS: bitrate_prescaler = MCP_8MHz_31k25BPS; break;
                case CAN_20KBPS:   bitrate_prescaler = MCP_8MHz_20kBPS;   break;
                case CAN_10KBPS:   bitrate_prescaler = MCP_8MHz_10kBPS;   break;
                case CAN_5KBPS:    bitrate_prescaler = MCP_8MHz_5kBPS;    break;
                default: break;
            }
            break;
        case MCP_16MHZ:
            switch (can_speed) {
                case CAN_1000KBPS: bitrate_prescaler = MCP_16MHz_1000kBPS; break;
                case CAN_500KBPS:  bitrate_prescaler = MCP_16MHz_500kBPS;  break;
                case CAN_250KBPS:  bitrate_prescaler = MCP_16MHz_250kBPS;  break;
                case CAN_200KBPS:  bitrate_prescaler = MCP_16MHz_200kBPS;  break;
                case CAN_125KBPS:  bitrate_prescaler = MCP_16MHz_125kBPS;  break;
                case CAN_100KBPS:  bitrate_prescaler = MCP_16MHz_100kBPS;  break;
                case CAN_95KBPS:   bitrate_prescaler = MCP_16MHz_95kBPS;   break;
                case CAN_83K3BPS:  bitrate_prescaler = MCP_16MHz_83k3BPS;  break;
                case CAN_80KBPS:   bitrate_prescaler = MCP_16MHz_80kBPS;   break;
                case CAN_50KBPS:   bitrate_prescaler = MCP_16MHz_50kBPS;   break;
                case CAN_40KBPS:   bitrate_prescaler = MCP_16MHz_40kBPS;   break;
                case CAN_33KBPS:   bitrate_prescaler = MCP_16MHz_33k3BPS;  break;
                case CAN_20KBPS:   bitrate_prescaler = MCP_16MHz_20kBPS;   break;
                case CAN_10KBPS:   bitrate_prescaler = MCP_16MHz_10kBPS;   break;
                case CAN_5KBPS:    bitrate_prescaler = MCP_16MHz_5kBPS;    break;
                default: break;
            }
            break;
        default: break;
    }
}

// returns: error count
uint8_t mcp2515_init(SPI_Regs *spi) {
    spi_inst = spi;
    
    DL_GPIO_setPins(GPIOA, SPI_PINS_CS_PIN);
    delay_cycles(16);

    mcp2515_reset();

    uint8_t canstat = mcp2515_read_register(MCP2515_CANSTAT);
    if(canstat != 0x80) return MCP2515_RESET_ERROR;

    // set prescalers
    mcp2515_write_register(MCP2515_CNF1, bitrate_prescaler.cnf1);
    mcp2515_write_register(MCP2515_CNF2, bitrate_prescaler.cnf2);
    mcp2515_write_register(MCP2515_CNF3, bitrate_prescaler.cnf3);

    // set message filters
    mcp2515_write_register(MCP2515_RXB0CTRL, 0x64); // RXB0CTRL
    mcp2515_write_register(MCP2515_RXB1CTRL, 0x60); // RXB1CTRL

    // configure interrupts 0x00 = disable, 0x03 = enable
    mcp2515_write_register(MCP2515_CANINTE, 0x03); // enable RX0IE and RX1IE

    // tx pins setup
    mcp2515_write_register(MCP2515_TXRTSCTRL, 0x00); // all as general purpose inputs

    // rx pins setup
    mcp2515_write_register(0x0C, 0x00); // disable both, or configure as interrupt pins

    // clear recieve buffer
    mcp2515_write_register(MCP2515_CANINTF, 0x00); // clear all interrupt flags
    
    // enter normal operating mode
    mcp2515_write_register(MCP2515_CANCTRL, MCP2515_CANSTAT_OPMOD_NORMAL);

    uint8_t mode;
    do {
        mode = mcp2515_read_register(MCP2515_CANSTAT);
    } while ((mode & 0xE0) != 0x00); // wait till it is in normal mode

    // enable interrupt controller for incoming can frames
    NVIC_ClearPendingIRQ(SPI_PINS_INT_IRQN);
    NVIC_EnableIRQ(SPI_PINS_INT_IRQN);
    delay_cycles(320000);
    
    return MCP2515_OK;
}

typedef enum { MCP2515_TXB0 = 0, MCP2515_TXB1, MCP2515_TXB2 } mcp2515_txb_t;

static const struct {
    uint8_t sidh;   // SIDH register address
    uint8_t ctrl;   // CTRL register address
} TXB_REGS[3] = {
    { MCP2515_TXB0SIDH, MCP2515_TXB0CTRL },
    { MCP2515_TXB1SIDH, MCP2515_TXB1CTRL },
    { MCP2515_TXB2SIDH, MCP2515_TXB2CTRL },
};

// Encode `frame` into TXBn's SIDH..DLC + data registers, set TXREQ, and
// report whether the post-RTS error bits flagged immediately.
static mcp2515_tx_status_t _send_to_buf(mcp2515_txb_t txbn, const mcp2515_frame_t *frame) {
    uint8_t buf[5 + CAN_MAX_DLEN];

    // ---- encode SIDH/SIDL/EID8/EID0 (reverse of mcp2515_read_frame's decode) ----
    if (frame->ext) {
        uint32_t id = frame->id;
        buf[MCP_SIDH] = (uint8_t)(id >> 21);
        buf[MCP_SIDL] = (uint8_t)(((id >> 13) & 0xE0) | TXB_EXIDE_MASK | ((id >> 16) & 0x03));
        buf[MCP_EID8] = (uint8_t)(id >> 8);
        buf[MCP_EID0] = (uint8_t)id;
    } else {
        uint32_t id = frame->id & 0x7FF;
        buf[MCP_SIDH] = (uint8_t)(id >> 3);
        buf[MCP_SIDL] = (uint8_t)((id << 5) & 0xE0);
        buf[MCP_EID8] = 0;
        buf[MCP_EID0] = 0;
    }

    buf[MCP_DLC] = (frame->dlc & DLC_MASK) | (frame->rtr ? 0x40 : 0); // RTR = bit 6 of DLC reg
    memcpy(&buf[5], frame->data, frame->dlc);

    mcp2515_write_registers(TXB_REGS[txbn].sidh, buf, 5 + frame->dlc);
    mcp2515_modify_register(TXB_REGS[txbn].ctrl, MCP2515_TXBCTRL_TXREQ, MCP2515_TXBCTRL_TXREQ);

    uint8_t ctrl = mcp2515_read_register(TXB_REGS[txbn].ctrl);
    if (ctrl & (MCP2515_TXBCTRL_ABTF | MCP2515_TXBCTRL_MLOA | MCP2515_TXBCTRL_TXERR))
        return MCP2515_TX_FAIL;
    return MCP2515_TX_OK;
}

// Send a CAN frame using the first free TX buffer (TXB0/1/2).
// Masks the MCP2515 GPIO INT IRQ for the duration of the SPI activity so the
// RX ISR (the only other thing on this SPI bus) can't preempt mid-transaction.
mcp2515_tx_status_t mcp2515_write_frame(const mcp2515_frame_t *frame) {
    if (frame->dlc > CAN_MAX_DLEN) return MCP2515_TX_FAIL;

    NVIC_DisableIRQ(SPI_PINS_INT_IRQN);
    __DSB(); __ISB();   // ensure mask takes effect before any SPI activity

    mcp2515_tx_status_t result = MCP2515_TX_ALL_BUSY;
    for (mcp2515_txb_t i = MCP2515_TXB0; i <= MCP2515_TXB2; i++) {
        uint8_t ctrl = mcp2515_read_register(TXB_REGS[i].ctrl);
        if ((ctrl & MCP2515_TXBCTRL_TXREQ) == 0) {
            result = _send_to_buf(i, frame);
            break;
        }
    }

    NVIC_EnableIRQ(SPI_PINS_INT_IRQN);
    return result;
}

int mcp2515_read(mcp2515_frame_t *frame) {
    return 0;
}

mcp2515_rx_status_t mcp2515_rxbuf_status(void) {
    uint8_t intf = mcp2515_read_register(MCP2515_CANINTF);
    if (intf & 0x01) return MCP2515_RX_BUF0; // RX0IF - message in RXB0
    if (intf & 0x02) return MCP2515_RX_BUF1; // RX1IF - message in RXB1
    return MCP2515_RX_NONE;
}

size_t mcp2515_available(void) {
    return mcp2515_ring_size(&msgbuf);
}

mcp2515_frame_t mcp2515_read_can(void) {
    mcp2515_frame_t frame = {0};
    mcp2515_frame_t *p = mcp2515_ring_peek(&msgbuf);
    if (p != NULL) {
        frame = *p;
        mcp2515_ring_advance(&msgbuf);
    }
    return frame;
}

// Read a pending frame from RXB0 or RXB1 and return it.
// On error (invalid rxbuf), returns a zeroed frame with dlc = 0.
// Clears the corresponding RXnIF flag so the INT pin re-arms.
mcp2515_frame_t mcp2515_read_frame(mcp2515_rx_status_t rxbuf) {
    mcp2515_frame_t frame = {0};

    uint8_t sidh_addr, ctrl_addr, data_addr, intf_mask;
    switch (rxbuf) {
        case MCP2515_RX_BUF0:
            sidh_addr = MCP2515_RXB0SIDH;
            ctrl_addr = MCP2515_RXB0CTRL;
            data_addr = MCP2515_RXB0D0;
            intf_mask = MCP2515_CANINTF_RX0IF;
            break;
        case MCP2515_RX_BUF1:
            sidh_addr = MCP2515_RXB1SIDH;
            ctrl_addr = MCP2515_RXB1CTRL;
            data_addr = MCP2515_RXB1D0;
            intf_mask = MCP2515_CANINTF_RX1IF;
            break;
        default:
            return frame;
    }

    // bulk-read SIDH, SIDL, EID8, EID0, DLC in one SPI transaction
    uint8_t tbufdata[5];
    mcp2515_read_registers(sidh_addr, tbufdata, 5);

    // build base 11-bit standard ID
    uint32_t id = ((uint32_t)tbufdata[MCP_SIDH] << 3) |
                  ((uint32_t)tbufdata[MCP_SIDL] >> 5);

    if ((tbufdata[MCP_SIDL] & TXB_EXIDE_MASK) == TXB_EXIDE_MASK) {
        // extended (29-bit) frame: append EID17:16 / EID15:8 / EID7:0
        id = (id << 2) | (tbufdata[MCP_SIDL] & 0x03);
        id = (id << 8) | tbufdata[MCP_EID8];
        id = (id << 8) | tbufdata[MCP_EID0];
        frame.ext = 1;
    }

    uint8_t dlc = tbufdata[MCP_DLC] & DLC_MASK;
    if (dlc > CAN_MAX_DLEN) dlc = CAN_MAX_DLEN;

    // RX-side RTR lives in the RXBnCTRL register, not in DLC
    uint8_t ctrl = mcp2515_read_register(ctrl_addr);
    frame.rtr = (ctrl & MCP2515_RXB0CTRL_RXRTR) ? 1 : 0;

    frame.id  = id;
    frame.dlc = dlc;

    // only read payload for data frames
    if (!frame.rtr && dlc > 0) {
        mcp2515_read_registers(data_addr, frame.data, dlc);
    }

    // clear just this buffer's RXnIF so the INT pin re-arms
    mcp2515_modify_register(MCP2515_CANINTF, intf_mask, 0x00);
    return frame;
}

// Drain any pending RX buffers into the ring. Safe to call from ISR.
void mcp2515_service_rx(void) {
    mcp2515_rx_status_t rxbuf;
    while ((rxbuf = mcp2515_rxbuf_status()) != MCP2515_RX_NONE) {
        mcp2515_frame_t frame = mcp2515_read_frame(rxbuf);
        mcp2515_ring_push(&msgbuf, &frame);
    }
}

void GROUP1_IRQHandler(void)
{
    DL_GPIO_clearInterruptStatus(GPIOB, SPI_PINS_MCP_INT_PIN);
    mcp2515_service_rx();
}