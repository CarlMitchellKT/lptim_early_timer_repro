/*
 * Reproducer: k_timer expires early / kernel time jumps on the STM32 LPTIM
 * tickless system timer (LPTIM1 on LSI, 4000 ticks/s per default).
 *
 * The host streams bursts of characters at 9600 baud into USART3 (ST-LINK
 * VCP). Every received character, in ISR context, restarts a one-shot k_timer
 * of K_USEC(RESTART_US). Characters arrive every ~1.04 ms, so during a burst
 * the timer must never expire; it may only expire after the last character of
 * a burst.
 *
 * k_timer_start() subtracts one tick from the duration (kernel/timer.c), so
 * e.g. K_USEC(1354) = 6 ticks legitimately expires 5..6 ticks (1250..1500 us)
 * after the call. MIN_LEGAL_US is that floor less 10% for the LSI's accuracy.
 * Build with -DRESTART_US=<us> to change the timer duration.
 *
 * Two independent checks use DWT->CYCCNT (core clock, unrelated to LPTIM/LSI):
 *   EARLY: the timer expired < MIN_LEGAL_US of real time after its restart.
 *   JUMP : the kernel cycle counter advanced > JUMP_US more than DWT between
 *          two consecutive RX ISRs.
 *
 * The main thread never uses a kernel timeout (it blocks on a semaphore given
 * from the RX ISR), so reporting does not depend on the timer under test and
 * the CPU idles in WFI between interrupts. Events are printed on the same UART
 * (polled TX). Sending 'S' makes the board print a STAT line.
 */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/printk.h>
#include <soc.h>
#include <stm32_ll_lptim.h>

#ifndef RESTART_US
#define RESTART_US 1250U
#endif
#define TICK_US      250U
/* k_timer_start() subtracts a tick: legal floor is (ceil(RESTART/tick) - 1) ticks, less 10% */
#define MIN_LEGAL_US ((((RESTART_US + TICK_US - 1) / TICK_US) - 1) * TICK_US * 9 / 10)
#define JUMP_US      500U /* > 2 ticks of disagreement between kernel and DWT */
#define CORE_HZ      DT_PROP(DT_NODELABEL(rcc), clock_frequency)

enum evt_kind
{
	EVT_EARLY,
	EVT_JUMP
};

struct evt {
	uint8_t kind;
	uint32_t dwt_us;  /* real time according to DWT */
	uint32_t kern_us; /* time according to k_cycle_get_32() */
	uint32_t cnt;     /* LPTIM1 CNT/ARR/ISR when the event was detected */
	uint32_t arr;
	uint32_t isr;
};

K_MSGQ_DEFINE(evq, sizeof(struct evt), 64, 4);
K_SEM_DEFINE(wake, 0, 1);

static const struct device *const uart = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
static struct k_timer one_shot;

static volatile uint32_t restart_dwt, restart_cyc;
static uint32_t prev_dwt, prev_cyc;
static bool have_prev;
static volatile bool stat_requested;
static volatile uint32_t n_rx, n_start, n_fire, n_early, n_jump, n_drop, n_rxerr;

static inline uint32_t dwt_to_us(uint32_t cycles)
{
	return (uint32_t)(((uint64_t)cycles * 1000000U) / CORE_HZ);
}

static inline uint32_t cyc_to_us(uint32_t cycles)
{
	return (uint32_t)(((uint64_t)cycles * 1000000U) / sys_clock_hw_cycles_per_sec());
}

static void post_event(uint8_t kind, uint32_t dwt_us, uint32_t kern_us)
{
	struct evt e = {
		.kind = kind,
		.dwt_us = dwt_us,
		.kern_us = kern_us,
		.cnt = LL_LPTIM_GetCounter(LPTIM1),
		.arr = LL_LPTIM_GetAutoReload(LPTIM1),
		.isr = LPTIM1->ISR,
	};

	if (k_msgq_put(&evq, &e, K_NO_WAIT) != 0) {
		n_drop++;
	}
	k_sem_give(&wake);
}

static void timer_expiry(struct k_timer *t)
{
	ARG_UNUSED(t);

	uint32_t dwt = DWT->CYCCNT;
	uint32_t cyc = k_cycle_get_32();
	uint32_t dwt_us = dwt_to_us(dwt - restart_dwt);
	uint32_t kern_us = cyc_to_us(cyc - restart_cyc);

	n_fire++;
	if (dwt_us < MIN_LEGAL_US) {
		n_early++;
		post_event(EVT_EARLY, dwt_us, kern_us);
	}
}

static void on_rx_char(void)
{
	uint32_t dwt = DWT->CYCCNT;
	uint32_t cyc = k_cycle_get_32();

	if (have_prev) {
		uint32_t kern_us = cyc_to_us(cyc - prev_cyc);
		uint32_t dwt_us = dwt_to_us(dwt - prev_dwt);

		/* kern_us < 30 ms keeps us clear of the 33.5 ms DWT wrap */
		if (kern_us < 30000U && kern_us > dwt_us + JUMP_US) {
			n_jump++;
			post_event(EVT_JUMP, dwt_us, kern_us);
		}
	}

	restart_dwt = DWT->CYCCNT;
	restart_cyc = k_cycle_get_32();
	k_timer_start(&one_shot, K_USEC(RESTART_US), K_NO_WAIT);
	n_start++;

	prev_dwt = DWT->CYCCNT;
	prev_cyc = k_cycle_get_32();
	have_prev = true;
}

static void uart_cb(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);
	uint8_t c;

	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
		if (uart_err_check(dev) != 0) {
			n_rxerr++;
		}
		if (uart_irq_rx_ready(dev)) {
			while (uart_fifo_read(dev, &c, 1) == 1) {
				if (c == 'S') {
					stat_requested = true;
					k_sem_give(&wake);
					continue;
				}
				n_rx++;
				on_rx_char();
			}
		}
	}
}

int main(void)
{
	struct evt e;
	uint32_t reset_csr = RCC->CSR;

	RCC->CSR |= RCC_CSR_RMVF;

	/* Debug aid: keep SWD attached (halt/inspect) while the core idles in WFI. */
	DBGMCU->CR |= DBGMCU_CR_DBG_SLEEP | DBGMCU_CR_DBG_STOP | DBGMCU_CR_DBG_STANDBY;

	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

	k_timer_init(&one_shot, timer_expiry, NULL);

	if (!device_is_ready(uart)) {
		return -1;
	}
	uart_irq_callback_user_data_set(uart, uart_cb, NULL);
	uart_irq_rx_enable(uart);

	printk("READY reset_csr=0x%08x core_hz=%u hw_cycles_hz=%u tick_hz=%u restart_us=%u "
	       "min_legal_us=%u\n",
	       reset_csr, CORE_HZ, sys_clock_hw_cycles_per_sec(), CONFIG_SYS_CLOCK_TICKS_PER_SEC,
	       RESTART_US, MIN_LEGAL_US);

	while (1) {
		k_sem_take(&wake, K_FOREVER);

		while (k_msgq_get(&evq, &e, K_NO_WAIT) == 0) {
			printk("EVT %s dwt_us=%u kern_us=%u cnt=%u arr=%u isr=0x%x\n",
			       e.kind == EVT_EARLY ? "early" : "jump", e.dwt_us, e.kern_us, e.cnt,
			       e.arr, e.isr);
		}
		if (stat_requested) {
			stat_requested = false;
			printk("STAT rx=%u starts=%u fires=%u early=%u jump=%u drop=%u rxerr=%u\n",
			       n_rx, n_start, n_fire, n_early, n_jump, n_drop, n_rxerr);
		}
	}
	return 0;
}
