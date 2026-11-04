Please help me implement a synchronized DAC output and ADC acquisition system using a Raspberry Pi Pico and a Raspberry Pi.

The Pico should continuously generate a sawtooth waveform through a DAC Click board based on the MCP4921. For every DAC output point, it should wait 10 microseconds for the output to settle, then take one ADC reading.

Use these parameters:
NUM_POINTS = 10000
DAC_MAX_VALUE = 4095
DAC_FULL_SCALE_VOLTAGE = 5.0
V_MIN = 0.7
V_MAX = 4.3

The hardware connections are:
- MCP4921 DAC Click connected to Pico pins GP9, GP10, and GP11. Specify the CS, SCK, and MOSI assignments.
- Analog input connected to Pico ADC pin GP26.
- Pico connected to a Raspberry Pi for data transfer and plotting. Pico and Pi connecting through USB.

The required behavior is:
1. Precompute one complete sawtooth containing exactly 10,000 points: 9,000 points for the rising portion from 0.7 V to 4.3 V, followed by 1,000 points for the falling portion back to 0.7 V.
2. For each point:
   - Write the corresponding DAC code to the MCP4921.
   - Wait at least 10 microseconds after the DAC output updates.
   - Read the ADC on GP26.
   - Store the DAC code and corresponding ADC reading at the same index in DAC[10000] and ADC[10000].
3. After each complete sawtooth, transfer both arrays to the Raspberry Pi while preserving their sample order and pairing.
4. On the Raspberry Pi, receive the arrays, convert the readings to voltages using the appropriate reference voltages and any input scaling, and plot both signals against sample index or time.
5. Repeat continuously. Since we use the same sawtooth waveform, the plot in the pi should look constant/fix looking.

Please give me the code to finish this work.


