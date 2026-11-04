Please update `pico_firmware/sawtooth_daq.c` and `pi_host/sawtooth_plot.py` so I can set the Pico’s trigger delay by manually selecting a time on the Pi’s plot.

**Current behavior:** The Pico sends the `DAC[]` and `ADC[]` arrays to the Pi, which plots them using `pi_host/sawtooth_plot.py`. The Pico’s trigger delay is hardcoded as `#define TRIGGER_DELAY_MS 50`.

**Requested changes:**

1. **Select a time:** In `pi_host/sawtooth_plot.py`, allow me to click on the displayed plot to select a time from the ADC plot.
2. **Send the selection:** Send the selected time, to the Pico through the existing Pi–Pico connection. Support fractional values such as **87.4 ms**.Pay attention to the unit to make sure consistency.
3. **Update the trigger delay:** In `pico_firmware/sawtooth_daq.c`, replace the hardcoded delay with a runtime variable that receives this value. Apply the updated delay to subsequent scans, triggering **GP28** using the existing timing reference and pulse behavior.

For example, selecting **x = 87.4 ms** in the plot shown in `example.png` should set the Pico’s trigger delay to **87.4 ms**.