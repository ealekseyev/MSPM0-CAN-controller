#include "ti_msp_dl_config.h"
#include "ti/driverlib/driverlib.h"
#include "ti/driverlib/m0p/dl_core.h"   // delay_cycles, POWER_STARTUP_DELAY
#include "ti/driverlib/dl_common.h"
#include "mcp2515/mcp2515.h"

void print(const char *str);
void print_hex(uint32_t val);
void print_hex8(uint8_t*, size_t);
void print_rx_frame(mcp2515_frame_t *f);
void process_cmd(void);
static uint8_t hex_nibble(char c);

static char uart_buf[32];
static uint8_t uart_buf_len = 0;

int main(void)
{
    SYSCFG_DL_init();
    print("initialized driverlib\n");

    mcp2515_set_bitrate(CAN_100KBPS, MCP_8MHZ);

    if(mcp2515_init(SPI_0_INST) == MCP2515_OK) print("initialized mcp2515\n");
    else print("mcp2515 initialization error\n");

    while(1) {
        while (mcp2515_available() > 0) {
            mcp2515_frame_t f = mcp2515_read_can();
            print_rx_frame(&f);
        }

        while (!DL_UART_isRXFIFOEmpty(UART_0_INST)) {
            char c = (char)DL_UART_receiveData(UART_0_INST);
            if (c == '\n' || c == '\r') {
                if (uart_buf_len > 0) { process_cmd(); uart_buf_len = 0; }
            } else if (uart_buf_len < sizeof(uart_buf) - 1) {
                uart_buf[uart_buf_len++] = c;
            }
        }

        delay_cycles(3200);
    }
}


/* CAN PASSTHROUGH */


static uint8_t hex_nibble(char c)
{
    if (c >= '0' && c <= '9') return (uint8_t)(c - '0');
    if (c >= 'A' && c <= 'F') return (uint8_t)(c - 'A' + 10);
    if (c >= 'a' && c <= 'f') return (uint8_t)(c - 'a' + 10);
    return 0;
}

void process_cmd(void)
{
    // Expected format: xxx:BBBBBBBB  (3-nibble ID, colon, up to 8 data bytes as hex pairs)
    if (uart_buf_len < 4 || uart_buf[3] != ':') return;

    mcp2515_frame_t f = {0};
    f.id = ((uint32_t)hex_nibble(uart_buf[0]) << 8)
         | ((uint32_t)hex_nibble(uart_buf[1]) << 4)
         |  (uint32_t)hex_nibble(uart_buf[2]);

    uint8_t data_chars = uart_buf_len - 4; // chars after the colon
    uint8_t nbytes = data_chars / 2;
    if (nbytes > 8) nbytes = 8;

    for (uint8_t i = 0; i < nbytes; i++) {
        f.data[i] = (hex_nibble(uart_buf[4 + i*2]) << 4)
                  |  hex_nibble(uart_buf[4 + i*2 + 1]);
    }
    f.dlc = nbytes;

    mcp2515_write_frame(&f);
}

void print_rx_frame(mcp2515_frame_t *f)
{
    const char *hex = "0123456789ABCDEF";
    char buf[6];

    // "RX: 0x" + 3-nibble ID
    print("RX: 0x");
    buf[0] = hex[(f->id >> 8) & 0xF];
    buf[1] = hex[(f->id >> 4) & 0xF];
    buf[2] = hex[(f->id >> 0) & 0xF];
    buf[3] = '\0';
    print(buf);

    // " [N]"
    print(" [");
    buf[0] = '0' + (f->dlc & 0xF);
    buf[1] = '\0';
    print(buf);
    print("]");

    // " BB BB ..."
    for (uint8_t i = 0; i < f->dlc; i++) {
        buf[0] = ' ';
        buf[1] = hex[(f->data[i] >> 4) & 0xF];
        buf[2] = hex[(f->data[i] >> 0) & 0xF];
        buf[3] = '\0';
        print(buf);
    }

    print("\n");
}


/* PRINTS */


void print(const char *str) {
    while (*str) {
        if (*str == '\n') {
            while (DL_UART_isTXFIFOFull(UART_0_INST));
            DL_UART_transmitData(UART_0_INST, '\r');
        }
        while (DL_UART_isTXFIFOFull(UART_0_INST));
        DL_UART_transmitData(UART_0_INST, (uint8_t)(*str));
        str++;
    }
}

void print_hex(uint32_t val) {
    char buf[11]; // "0x" + 8 hex digits + \n + null
    const char *hex = "0123456789ABCDEF";

    buf[0]  = '0';
    buf[1]  = 'x';
    buf[2]  = hex[(val >> 28) & 0xF];
    buf[3]  = hex[(val >> 24) & 0xF];
    buf[4]  = hex[(val >> 20) & 0xF];
    buf[5]  = hex[(val >> 16) & 0xF];
    buf[6]  = hex[(val >> 12) & 0xF];
    buf[7]  = hex[(val >>  8) & 0xF];
    buf[8]  = hex[(val >>  4) & 0xF];
    buf[9]  = hex[(val >>  0) & 0xF];
    buf[10] = '\0';

    print(buf);
}


void print_hex8(uint8_t* val, size_t len) {
    char buf[50]; // "0x" + 8 hex digits + \n + null
    const char *hex = "0123456789ABCDEF";

    buf[0]  = '0';
    buf[1]  = 'x';
    size_t bufind = 2;
    if(len == 0) {
        buf[bufind++] = '0';
    } else {
        for(size_t i = 0; i < len; i++) {
            buf[bufind++]  = hex[(val[i] >> 4) & 0xF];
            buf[bufind++]  = hex[val[i] & 0xF];
        }
    }
    buf[bufind] = '\0';

    print(buf);
}
