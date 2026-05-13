# CANController

MCP2515 CAN controller driver for the TI MSPM0 (MSPM0G3507). Talks to the chip over SPI, handles RX via the INT pin into a ring buffer, and TX across all three TX buffers.

Driver lives in `mcp2515/`. Drop `mcp2515.c` / `mcp2515.h` into your project, call `mcp2515_set_bitrate()` + `mcp2515_init()`, then `mcp2515_send_can()` / `mcp2515_read_can()`.

## Transmission monitor

`client/transmission_monitor.py` reads the same serial output as the dashboard and prints raw CAN frames that look relevant to transmission state.

```powershell
python client\transmission_monitor.py COM3
python client\transmission_monitor.py COM3 --scan-all
python client\transmission_monitor.py --replay log.txt --scan-all
```

By default it watches `0x304` byte 0 using the current dashboard values: Park `E3`, Reverse `C2`, Neutral `D1`, Drive `C5`-`CA`. In Drive, `C5` is gear 1, `C6` is gear 2, through `CA` as gear 6. Use `--scan-all` while shifting through P/R/N/D to find other IDs carrying those bytes.
