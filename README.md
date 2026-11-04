# Pico–Pi Sawtooth Data Acquisition

## Overview

The Pico firmware in `pico_firmware` generates 10,000 DAC values (`DAC[10000]`), acquires 10,000 ADC samples (`ADC[10000]`), and sends both arrays to the Raspberry Pi over **USB**. The Pi displays the data using `pi_host/sawtooth_plot.py`.

## Pico Wiring

| Pico signal | Physical pin | Connection |
| --- | --- | --- |
| GP9 | 12 | DAC Click: CS |
| GP10 | 14 | DAC Click: SCK |
| GP11 | 15 | DAC Click: SDI |
| VBUS | 40 | DAC Click: 5V |
| GND | 38 | DAC Click: GND |
| GP28 | 34 | Camera trigger output, as shown in `design.png` |
| GP26 / ADC0 | 31 | Analog input signal |
| AGND | 33 | Analog input ground |

## Interactive Trigger Control (`cavity_mode_v2`)

### Pi Host: `pi_host/sawtooth_plot.py`

- **Select a time:** Left-click on the plot to choose a trigger delay in milliseconds. The script sends the selected x-coordinate exactly as displayed in the plot toolbar.
- **Confirm the delay:** A green line marks the delay confirmed by the Pico. A dashed line marks the selected time while confirmation is pending.
- **Restore after a reset:** The Pico returns to its default delay of **50 ms** after a reset. The script resends your selected delay when it reconnects.
- **Set the startup delay:** Use `--trigger-ms 87.4` to start with a trigger delay of **87.4 ms**.

### Pico Firmware: `pico_firmware/sawtooth_daq.c`

- The fixed definition `#define TRIGGER_DELAY_MS 50` has been replaced with a runtime variable, initialized to **50 ms** and updated by the Pi.
- Each new delay takes effect at the start of the **next scan**. GP28 uses the existing trigger timing reference and pulse behavior.
- Each data frame reports the delay used for that scan so the Pi can confirm the update.

## Build and Flash the Pico Firmware

Run the following commands from inside the `pico_firmware` directory:

```bash
# Create and enter the build directory.
mkdir -p build
cd build

# Configure the build using CMakeLists.txt in the parent directory.
cmake ..

# Compile the firmware using four parallel jobs.
make -j4
```

Build outputs are stored in `build`. These include `sawtooth_daq.elf` and, depending on the build configuration, additional files such as `.uf2` and `.bin`.

From the same `build` directory, program the Pico using OpenOCD:

```bash
sudo openocd \
  -f interface/cmsis-dap.cfg \
  -f target/rp2040.cfg \
  -c "adapter speed 5000" \
  -c "program sawtooth_daq.elf verify reset exit"
```

## Run the Pi Plotting Script

Run the following command from inside the `pi_host` directory:

```bash
python3 sawtooth_plot.py
```

To set an initial trigger delay of **87.4 ms**, use:

```bash
python3 sawtooth_plot.py --trigger-ms 87.4
```

## Example Plot

![Example DAC and ADC plot](https://github.com/user-attachments/assets/b1d0f373-97a0-4d61-8f35-ac7ef40366d1)
