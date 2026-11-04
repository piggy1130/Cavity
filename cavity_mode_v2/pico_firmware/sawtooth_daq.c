/*
 * sawtooth_daq.c - synchronized DAC sweep + ADC acquisition, Raspberry Pi Pico (RP2040)
 *
 * Core 1 (timing-critical, no interrupts, runs from SRAM):
 *   for each of the 10,000 precomputed sawtooth points, on a fixed 18 us time slot:
 *     1. write the DAC code to the MCP4921 (DAC Click) over SPI1
 *     2. wait >= 10 us after the DAC output updates (CS rising edge)
 *     3. read ADC0 (GP26)
 *     4. store the code in DAC[i] and the reading in ADC[i] (same index i)
 *   then hand the finished frame to core 0 and start the next sawtooth right away,
 *   so the DAC waveform is continuous and strictly periodic (180 ms per sawtooth).
 *
 * Core 0:
 *   sends every finished frame to the Raspberry Pi over USB (CDC serial) as one binary
 *   packet:  36-byte header | DAC[10000] uint16 | ADC[10000] uint16 | CRC-32
 *   If the host falls behind, whole sawtooth frames are skipped (never torn apart);
 *   the DAC sweep itself is never paused.
 *   It also reads commands from the Pi on the same USB serial link, one ASCII line each:
 *     TRIG_DELAY_US=<whole microseconds>\n     e.g. TRIG_DELAY_US=87400  ->  87.4 ms
 *   A new GP28 trigger delay is used from the next sawtooth on, and every frame header
 *   reports the delay that sawtooth actually used, so the Pi can confirm it.
 *
 * Wiring (see README.md)
 *   Pico GP9  (pin 12) -> DAC Click CS    chip select (driven as plain GPIO)
 *   Pico GP10 (pin 14) -> DAC Click SCK   SPI1 SCK
 *   Pico GP11 (pin 15) -> DAC Click SDI   SPI1 TX = MOSI
 *   Pico VBUS (pin 40) -> DAC Click 5V    PWR SEL jumper = +5V, REFERENCE jumper = VCC
 *   Pico GND  (pin 38) -> DAC Click GND
 *   Pico GP28 (pin 34) -> trigger output, high trigger_delay_us after the start of each sawtooth
 *   analog signal     -> Pico GP26 (pin 31, ADC0); signal ground -> AGND (pin 33)
 *   GP26 must stay within 0..3.3 V. The sweep tops out at V_MAX = 3.2 V, so DAC VOUT
 *   can be wired straight to GP26 for a loop-back test.
 */

#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/stdio_usb.h"
#include "pico/stdio/driver.h"
#include "hardware/adc.h"
#include "hardware/clocks.h"
#include "hardware/spi.h"
#include "hardware/structs/bus_ctrl.h"
#include "hardware/sync.h"

// ------------------------------------------------------------------ waveform
#define NUM_POINTS              10000
#define NUM_RISE_POINTS         9000
#define NUM_FALL_POINTS         (NUM_POINTS - NUM_RISE_POINTS)   // 1000
#define DAC_MAX_VALUE           4095
#define DAC_FULL_SCALE_VOLTAGE  5.0
#define V_MIN                   0.7
#define V_MAX                   3.2   // kept below the ADC's 3.3 V limit so GP26 can read
                                      // the DAC directly over the whole sweep

// ------------------------------------------------------------------ timing
#define SETTLE_US        10   // minimum wait after each DAC update before the ADC read
#define POINT_PERIOD_US  18   // fixed spacing of DAC updates -> 10000 * 18 us = 180 ms
                              // per sawtooth. One point needs about 15 us:
                              // SPI ~2 us + settle 10 us + ADC conversion 2 us + overhead.
                              // If the Pi shows "late pts" > 0, raise this; if it stays 0
                              // you can try 16 (160 ms).

// ------------------------------------------------------------------ pins / peripherals
#define DAC_SPI          spi1
#define DAC_SPI_HZ       10000000u  // requested; SDK rounds down (8.9 MHz). MCP4921 max 20 MHz
#define PIN_DAC_CS       9
#define PIN_DAC_SCK      10
#define PIN_DAC_MOSI     11
#define PIN_ADC          26
#define ADC_INPUT        0          // GP26 = ADC0
#define PIN_TRIGGER      28         // GP28 output: pulses high once per sawtooth

// ------------------------------------------------------------------ trigger output (GP28)
// t0 = time of point 0 of each sawtooth. GP28 goes high trigger_delay_us after t0 and
// stays high for TRIGGER_PULSE_US. Fired once per sawtooth (every 180 ms).
// trigger_delay_us is a run-time variable set by the Pi (TRIG_DELAY_US=... command). It is
// kept in whole microseconds, the unit of the Pico timer, so fractional milliseconds are
// exact: 87.4 ms = 87400 us. After a reset it starts at TRIGGER_DELAY_DEFAULT_US.
#define TRIGGER_DELAY_DEFAULT_US 50000u   // 50 ms until the Pi sends a delay
#define TRIGGER_PULSE_US         1000
// GP28 is only switched while waiting for points 1 .. NUM_POINTS-1, so the whole pulse has
// to fit before the last point: 9997 * 18 - 10 = 179936 us. Larger values are ignored.
#define TRIGGER_DELAY_MAX_US     ((NUM_POINTS - 3u) * POINT_PERIOD_US - TRIGGER_PULSE_US)

// MCP4921 16-bit command: bit15 A/B=0 (write DAC), bit14 BUF=0 (unbuffered VREF, needed
// when VREF = VDD), bit13 GA=1 (gain 1x), bit12 SHDN=1 (output on), bits 11..0 = code.
#define MCP4921_WRITE    0x3000u

// ------------------------------------------------------------------ frame sent to the Pi
#define FRAME_MAGIC      0x46574153u   // bytes 'S','A','W','F' on the wire (little-endian)

typedef struct {
    uint32_t magic;
    uint32_t frame_id;          // sawtooth number since boot (a gap = sawtooth not sent)
    uint32_t num_points;
    uint32_t point_period_us;
    uint32_t settle_us;
    uint32_t t0_us;             // timer value (us) of point 0 of this sawtooth
    uint32_t overruns;          // points in this sawtooth that started late (expect 0)
    uint32_t dropped;           // sawtooths not sent since boot because USB was busy
    uint32_t trigger_delay_us;  // GP28 delay after t0 (us) used for this sawtooth
    uint16_t dac[NUM_POINTS];   // DAC code written for point i
    uint16_t adc[NUM_POINTS];   // ADC reading taken after DAC[i] had settled
    uint32_t crc32;             // CRC-32 (same as Python zlib.crc32) of everything above
} frame_t;

static_assert(sizeof(frame_t) == 36 + 4 * NUM_POINTS + 4, "frame_t must not contain padding");
static_assert(POINT_PERIOD_US >= SETTLE_US + 6, "time slot too short for SPI + settle + ADC");
static_assert(TRIGGER_DELAY_DEFAULT_US <= TRIGGER_DELAY_MAX_US,
              "default trigger delay + pulse must fall inside one sawtooth");

// Everything below lives in SRAM (copy_to_ram build); nothing is const so the table stays in RAM.
static uint16_t dac_table[NUM_POINTS];      // one complete sawtooth, precomputed
static frame_t frames[2];                   // double buffer: core 1 fills one, core 0 sends one
static volatile bool frame_free[2] = {true, true};
// GP28 trigger delay in us after t0: written by core 0 when the Pi sends one, read by core 1
// once per sawtooth. An aligned 32-bit load/store is atomic on the RP2040, so no lock needed.
static volatile uint32_t trigger_delay_us = TRIGGER_DELAY_DEFAULT_US;
static uint32_t settle_cycles;              // SETTLE_US expressed in clk_sys cycles
static uint32_t crc_table[256];

// ------------------------------------------------------------------ sawtooth table
static uint16_t volts_to_code(double v) {
    double c = v / DAC_FULL_SCALE_VOLTAGE * DAC_MAX_VALUE + 0.5;   // round to nearest
    if (c < 0.0) c = 0.0;
    if (c > DAC_MAX_VALUE) c = DAC_MAX_VALUE;
    return (uint16_t)c;
}

static void build_sawtooth(void) {
    // Rising: 9000 points from V_MIN to V_MAX, both ends included (index 0 .. 8999).
    for (int i = 0; i < NUM_RISE_POINTS; i++) {
        double v = V_MIN + (V_MAX - V_MIN) * i / (NUM_RISE_POINTS - 1);
        dac_table[i] = volts_to_code(v);
    }
    // Falling: 1000 points from one step below V_MAX down to V_MIN (index 9000 .. 9999);
    // the peak is not repeated.
    for (int j = 0; j < NUM_FALL_POINTS; j++) {
        double v = V_MAX - (V_MAX - V_MIN) * (j + 1) / NUM_FALL_POINTS;
        dac_table[NUM_RISE_POINTS + j] = volts_to_code(v);
    }
}

// ------------------------------------------------------------------ CRC-32 (IEEE, as zlib)
static void crc32_init(void) {
    for (uint32_t i = 0; i < 256; i++) {
        uint32_t c = i;
        for (int k = 0; k < 8; k++) c = (c & 1u) ? 0xEDB88320u ^ (c >> 1) : c >> 1;
        crc_table[i] = c;
    }
}

static uint32_t crc32_ieee(const void *data, size_t len) {
    const uint8_t *p = data;
    uint32_t c = 0xFFFFFFFFu;
    while (len--) c = crc_table[(c ^ *p++) & 0xFFu] ^ (c >> 8);
    return c ^ 0xFFFFFFFFu;
}

// ------------------------------------------------------------------ hardware
static void dac_init(void) {
    spi_init(DAC_SPI, DAC_SPI_HZ);
    spi_set_format(DAC_SPI, 16, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);   // MCP4921: mode 0,0
    gpio_set_function(PIN_DAC_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_DAC_MOSI, GPIO_FUNC_SPI);
    gpio_init(PIN_DAC_CS);
    gpio_put(PIN_DAC_CS, 1);            // set the latch high first so CS never glitches low
    gpio_set_dir(PIN_DAC_CS, GPIO_OUT);
}

static void adc_setup(void) {
    adc_init();
    adc_gpio_init(PIN_ADC);             // analog mode: digital input buffer and pulls off
    adc_select_input(ADC_INPUT);
}

// LDAC is tied to GND on the DAC Click, so VOUT updates on the CS rising edge.
static inline void __not_in_flash_func(dac_write)(uint16_t code) {
    const uint16_t word = (uint16_t)(MCP4921_WRITE | (code & 0x0FFFu));
    gpio_put(PIN_DAC_CS, 0);
    spi_write16_blocking(DAC_SPI, &word, 1);   // returns after the last bit has left the shifter
    gpio_put(PIN_DAC_CS, 1);                   // <- DAC output updates here
}

// ------------------------------------------------------------------ core 1: acquisition
static void __not_in_flash_func(core1_acquire)(void) {
    uint32_t frame_id = 0;
    uint32_t dropped = 0;
    unsigned b = 0;                              // buffer the next sawtooth goes into
    uint32_t t_slot = time_us_32() + 1000;       // start time of the next point

    for (;;) {
        // Fill buffer b if core 0 has finished sending it; otherwise still run the sawtooth
        // (the DAC must not pause) but discard the readings.
        frame_t *f = frame_free[b] ? &frames[b] : NULL;
        __dmb();
        uint32_t t0 = 0;
        uint32_t overruns = 0;
        // Latch the delay once per sawtooth: a value the Pi sends mid-sweep takes effect with
        // the next sawtooth, never halfway through one.
        const uint32_t delay_us = trigger_delay_us;

        // GP28 trigger state for this sawtooth: 0 = waiting, 1 = pulse high, 2 = done.
        // Checked inside the slot-wait spin, so the DAC/ADC timing itself is untouched.
        unsigned trig_state = 0;
        uint32_t trig_at = 0;                         // time GP28 should go high
        uint32_t trig_off_at = 0;                     // time GP28 should go low again

        for (uint32_t i = 0; i < NUM_POINTS; i++) {
            // Start each point on its own fixed time slot -> evenly spaced samples.
            if ((int32_t)(time_us_32() - t_slot) > 0) overruns++;
            while ((int32_t)(time_us_32() - t_slot) < 0) {
                const uint32_t now = time_us_32();
                if (trig_state == 0 && i > 0 && (int32_t)(now - trig_at) >= 0) {
                    gpio_put(PIN_TRIGGER, 1);
                    trig_off_at = now + TRIGGER_PULSE_US;
                    trig_state = 1;
                } else if (trig_state == 1 && (int32_t)(now - trig_off_at) >= 0) {
                    gpio_put(PIN_TRIGGER, 0);
                    trig_state = 2;
                }
                tight_loop_contents();
            }
            if (i == 0) {
                t0 = time_us_32();                    // actual (not scheduled) start time
                trig_at = t0 + delay_us;
            }
            t_slot += POINT_PERIOD_US;

            const uint16_t code = dac_table[i];
            dac_write(code);                          // 1. DAC output updates
            busy_wait_at_least_cycles(settle_cycles); // 2. wait >= 10 us
            const uint16_t raw = adc_read();          // 3. one ADC conversion (~2 us)
            if (f) {                                  // 4. same index -> paired
                f->dac[i] = code;
                f->adc[i] = raw;
            }
        }

        if (f) {
            f->magic = FRAME_MAGIC;
            f->frame_id = frame_id;
            f->num_points = NUM_POINTS;
            f->point_period_us = POINT_PERIOD_US;
            f->settle_us = SETTLE_US;
            f->t0_us = t0;
            f->overruns = overruns;
            f->dropped = dropped;
            f->trigger_delay_us = delay_us;
            __dmb();                                 // frame contents visible before handoff
            frame_free[b] = false;
            multicore_fifo_push_blocking(b);          // never blocks: at most 2 in flight
            b ^= 1u;
        } else {
            dropped++;
        }
        frame_id++;
    }
}

// ------------------------------------------------------------------ core 0: USB transfer
static void send_frame(frame_t *f) {
    if (!stdio_usb_connected()) return;       // nobody has the port open on the Pi
    f->crc32 = crc32_ieee(f, offsetof(frame_t, crc32));
    // Raw write straight to the USB CDC driver: no CR/LF translation, whole frame at once.
    stdio_usb.out_chars((const char *)f, (int)sizeof(*f));
}

// ------------------------------------------------------------------ core 0: commands from the Pi
// One ASCII line per command, ended by '\n' (or '\r'):
//   TRIG_DELAY_US=<0 .. TRIGGER_DELAY_MAX_US>   GP28 delay after t0, whole microseconds
// Unknown lines and out-of-range values are ignored (the delay stays as it was); the Pi sees
// the delay in use in every frame header.
#define CMD_MAX_LEN 32

static void handle_command(const char *line) {
    static const char key[] = "TRIG_DELAY_US=";
    if (strncmp(line, key, sizeof key - 1) != 0) return;
    const char *p = line + sizeof key - 1;
    if (*p == '\0') return;
    uint32_t us = 0;
    for (; *p != '\0'; p++) {
        if (*p < '0' || *p > '9') return;
        us = us * 10u + (uint32_t)(*p - '0');
        if (us > TRIGGER_DELAY_MAX_US) return;   // out of range (also rules out overflow)
    }
    trigger_delay_us = us;                       // core 1 uses it from the next sawtooth
}

// Non-blocking: handles whatever the Pi has sent so far.
static void poll_host_commands(void) {
    static char line[CMD_MAX_LEN];
    static unsigned len = 0;
    static bool too_long = false;
    char buf[64];
    int n;
    while ((n = stdio_usb.in_chars(buf, (int)sizeof buf)) > 0) {
        for (int k = 0; k < n; k++) {
            const char c = buf[k];
            if (c == '\n' || c == '\r') {
                line[len] = '\0';
                if (len > 0 && !too_long) handle_command(line);
                len = 0;
                too_long = false;
            } else if (len < CMD_MAX_LEN - 1) {
                line[len++] = c;
            } else {
                too_long = true;                 // drop the whole line
            }
        }
    }
}

int main(void) {
    stdio_init_all();                         // USB CDC -> /dev/ttyACM0 on the Raspberry Pi

#ifdef PICO_SMPS_MODE_PIN
    // Put the on-board regulator in PWM mode: less ripple on 3V3 (= ADC_VREF) -> less ADC noise.
    gpio_init(PICO_SMPS_MODE_PIN);
    gpio_put(PICO_SMPS_MODE_PIN, 1);
    gpio_set_dir(PICO_SMPS_MODE_PIN, GPIO_OUT);
#endif

    build_sawtooth();
    crc32_init();
    dac_init();
    adc_setup();
    gpio_init(PIN_TRIGGER);
    gpio_put(PIN_TRIGGER, 0);
    gpio_set_dir(PIN_TRIGGER, GPIO_OUT);
    settle_cycles = SETTLE_US * (clock_get_hz(clk_sys) / 1000000u);

    // Core 1 wins every SRAM / bus conflict with core 0, so the CRC + USB work on core 0
    // cannot stretch the timing loop on core 1.
    bus_ctrl_hw->priority = BUSCTRL_BUS_PRIORITY_PROC1_BITS;

    multicore_launch_core1(core1_acquire);

    for (;;) {
        poll_host_commands();                                     // new trigger delay?
        uint32_t b;                                               // index of a finished frame
        if (!multicore_fifo_pop_timeout_us(1000, &b)) continue;   // none yet
        __dmb();
        send_frame(&frames[b]);
        __dmb();
        frame_free[b] = true;                                     // core 1 may refill it
    }
}
