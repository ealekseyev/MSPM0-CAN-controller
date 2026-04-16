#include "ti_msp_dl_config.h"
#include "ti/driverlib/driverlib.h"
#include "ti/driverlib/m0p/dl_core.h"   // delay_cycles, POWER_STARTUP_DELAY
#include "ti/driverlib/dl_common.h"
#include "mcp2515/mcp2515.h"

void print(const char *str);
void print_hex(uint32_t val);
void print_hex8(uint8_t*, size_t);

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
            print("CAN RX: ID ");
            print_hex(f.id);
            print(" ");
            print_hex8(f.data, 8);
            print("\n");

            f.id = 0x021;
            f.data[0] = 0xBE;
            f.data[1] = 0xEF;
            f.data[2] = 0xDE;
            f.data[3] = 0xAD;
            f.data[4] = 0x00;
            f.data[5] = 0x0B;
            f.data[6] = 0xAD;
            f.dlc = 7;
            mcp2515_write_frame(&f);
        }
        delay_cycles(3200);
    }
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
