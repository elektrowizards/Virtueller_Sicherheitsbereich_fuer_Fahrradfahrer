# Local patches to Adafruit BNO08x 1.2.7

This is a copy of https://github.com/adafruit/Adafruit_BNO08x (1.2.7),
including the CEVA SH-2 sources, with two changes. Everything else is
unchanged. Licences: see license.txt and the headers of the sh2 files.

1. `src/shtp.c`, `txProcess()`: the write was retried forever while the
   I2C write failed. Now at most `SHTP_TX_MAX_RETRIES` (20) retries, then
   `SH2_ERR_IO` is returned.
   Bench symptom: after a BNO085 reset the firmware hung for good in
   `enableReport()` (main loop, button and motor control frozen).

2. `src/sh2.c`, `opProcess()`: operations with `timeout_us == 0` (all of
   them in this version) waited forever for the hub's reply. Now a default
   limit of `SH2_OP_DEFAULT_TIMEOUT_US` (0.5 s) applies; on timeout the
   operation ends with `SH2_ERR_TIMEOUT` as in the original code path.
