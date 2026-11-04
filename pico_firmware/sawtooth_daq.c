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
 *   packet:  32-byte header | DAC[10000] uint16 | ADC[10000] uint16 | CRC-32
 *   If the host falls behind, whole sawtooth frames are skipped (never torn apart);
 *   the DAC sweep itself is never paused.
 *
 * Wiring (see README.md)
 *   Pico GP9  (pin 12) -> DAC Click CS    chip select (driven as plain GPIO)
 *   Pico GP10 (pin 14) -> DAC Click SCK   SPI1 SCK
 *   Pico GP11 (pin 15) -> DAC Click SDI   SPI1 TX = MOSI
 *   Pico VBUS (pin 40) -> DAC Click 5V    PWR SEL jumper = +5V, REFERENCE jumper = VCC
 *   Pico GND  (pin 38) -> DAC Click GND
 *   analog signal      -> Pico GP26 (pin 31, ADC0); signal ground -> AGND (pin 33)
 *   GP26 must stay within 0..3.3 V. The sweep tops out at V_MAX = 3.2 V, so DAC VOUT
 *   can be wired straight to GP26 for a loop-back test.
 */

#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

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
    uint16_t dac[NUM_POINTS];   // DAC code written for point i
    uint16_t adc[NUM_POINTS];   // ADC reading taken after DAC[i] had settled
    uint32_t crc32;             // CRC-32 (same as Python zlib.crc32) of everything above
} frame_t;

static_assert(sizeof(frame_t) == 32 + 4 * NUM_POINTS + 4, "frame_t must not contain padding");
static_assert(POINT_PERIOD_US >= SETTLE_US + 6, "time slot too short for SPI + settle + ADC");

// Everything below lives in SRAM (copy_to_ram build); nothing is const so the table stays in RAM.
static uint16_t dac_table[NUM_POINTS];      // one complete sawtooth, precomputed
static frame_t frames[2];                   // double buffer: core 1 fills one, core 0 sends one
static volatile bool frame_free[2] = {true, true};
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

        for (uint32_t i = 0; i < NUM_POINTS; i++) {
            // Start each point on its own fixed time slot -> evenly spaced samples.
            if ((int32_t)(time_us_32() - t_slot) > 0) overruns++;
            while ((int32_t)(time_us_32() - t_slot) < 0) tight_loop_contents();
            if (i == 0) t0 = time_us_32();            // actual (not scheduled) start time
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
            __dmb();                                  // frame contents visible before handoff
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
    settle_cycles = SETTLE_US * (clock_get_hz(clk_sys) / 1000000u);

    // Core 1 wins every SRAM / bus conflict with core 0, so the CRC + USB work on core 0
    // cannot stretch the timing loop on core 1.
    bus_ctrl_hw->priority = BUSCTRL_BUS_PRIORITY_PROC1_BITS;

    multicore_launch_core1(core1_acquire);

    for (;;) {
        const uint32_t b = multicore_fifo_pop_blocking();   // index of a finished frame
        __dmb();
        send_frame(&frames[b]);
        __dmb();
        frame_free[b] = true;                                // core 1 may refill it
    }
}
