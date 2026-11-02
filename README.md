# STM32 LPTIM tickless timer: early k_timer expiry / kernel-time jumps

Minimal reproducer for `drivers/timer/stm32_lptim_timer.c` on the stock
`b_g474e_dpow1` and `nucleo_f413zh` boards (LPTIM1 on LSI 32 kHz).
Tree under test: Zephyr v3.7.3-rc1 (`workspace/zephyr`).

## What it does

* Firmware (`src/main.c`): every character received on USART3 restarts a
  one-shot `k_timer` from the RX ISR. Two checks use `DWT->CYCCNT`, which does
  not depend on LPTIM/LSI:
  * **early**: timer expired sooner than `k_timer_start()` allows (real time).
  * **jump**: `k_cycle_get_32()` advanced > 500 us more than DWT between two
    consecutive RX ISRs.
  Send `S` to get a `STAT` line with the counters.
* `test_lptim_early_timer.py`: streams bursts of 5-20 back-to-back characters
  with a gap, reads the counters, exits 1 if the bug is seen.

## Run

```bash
# With this folder in your `west` workspace
west build -p always -b b_g474e_dpow1 lptim_early_timer_repro -d build_lptim_repro

uv run --script test_lptim_early_timer.py
```

Exit status: 0 no anomaly, 1 bug reproduced, 2 setup problem.

## Results (B-G474E-DPOW1, 15-20 s runs, ~8-11k timer restarts each)

| Build | Frame / char period | Result |
|-------|---------------------|--------|
| default, `K_USEC(1250)` (5 ticks) | 8N1 / 1.04 ms | 5/5 reproduce: ~1000 early expiries, ~960 time jumps per run |
| `-DCMAKE_C_FLAGS=-DRESTART_US=1354` + `-DEXTRA_DTC_OVERLAY_FILE=boards/frame_8e2.overlay`, `--frame 8E2` | 8E2 / 1.25 ms | 3/3 reproduce: ~560 early, ~560 jumps per run |
| `RESTART_US=1100`, `1200` | 8N1 | reproduce |
| `RESTART_US=1000` (4 ticks), `1300`, `1354` (6 ticks) | 8N1 | clean (no anomaly in 11k-33k restarts) |
| default, but `CONFIG_STM32_LPTIM_TIMER=n CONFIG_CORTEX_M_SYSTICK=y` | 8N1 | 0 anomalies in 3 runs (control) |

### NUCLEO-F413ZH (STM32F413, Cortex-M4 96 MHz)

Zephyr has no LPTIM node for STM32F4, so `boards/nucleo_f413zh.overlay` defines
LPTIM1 (0x40002400, IRQ 97, clock from LSI via `RCC_DCKCFGR2.LPTIM1SEL`) as
`stm32_lp_tick_source`, sets USART3 (ST-LINK VCP) to 9600 baud and deletes
`cpu-power-states` (same STOP caveat as above). Build with `-b nucleo_f413zh`,
flash with openocd `target/stm32f4x.cfg` (use `connect_assert_srst`; the core
sleeps in WFI) and the ST-LINK V2-1 VCP is auto-detected too (`--port` if
several boards are attached).

| Build | Result |
|-------|--------|
| default (LPTIM, `K_USEC(1250)`, 8N1) | 3/3 reproduce: ~450-500 early, ~450-500 jumps per 15 s run (~7k restarts) |
| `CONFIG_STM32_LPTIM_TIMER=n CONFIG_CORTEX_M_SYSTICK=y` (control) | 0 anomalies in 2 runs |

So the anomaly is not specific to STM32G4. Same signature as on the G4
(`early dwt=91us kernel=1093us`, `jump dwt=1024us kernel=2000us`).

Typical signature: `early dwt_us=85 kern_us=1093` (timer fires 85 us of real
time after restart while kernel time advanced ~1.1 ms) and `jump dwt_us=1025
kern_us=2031` (kernel time advanced ~1 ms more than real time).

## Findings and caveats

* The anomaly needs the restart to land within a few counts of the current ARR
  (Auto Reload Register value).
  For an N-tick timer (`K_USEC` rounded up) the driver sets `ARR = 8*floor(CNT/8) + 8N - 1`,
  i.e. ARR is 32-39 counts ahead of CNT for N=5 and 40-47 for N=6. A 1.04 ms
  character period is 33.3 counts, so it hits the 5-tick window; a 6-tick timer
  needs a ~1.25 ms period. Real systems may hit this issue with longer windows
  through ISR jitter or other kernel timer activity.
* `k_timer_start()` subtracts one tick from the duration (`kernel/timer.c`), so
  `K_USEC(1354)` may legitimately fire 5-6 ticks (1250-1500 us) after the call.
  The "early" threshold is that floor minus 10%, to account for LSI inaccuracy.
* **I've not root-caused this.** The jump is about one ARR period (32-40
  counts), consistent with a double-counted or mis-read ARR, but this data does
  not distinguish suspected mechanism.
* The main thread blocks on a semaphore (no kernel timeouts), so reporting does
  not depend on the timer under test. Event lines go out at 9600 baud
  (~70 ms each), so counters are authoritative.
