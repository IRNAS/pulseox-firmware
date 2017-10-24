# Calibration method

Use the patient simulator that enables pulse oximetry simulation. 

0. Flash the debug firmware (debug_firmware.bin) onto your Glia pulse oximeter as shown in [these instructions](https://github.com/IRNAS/pulseox-hardware/blob/v2.x/testing%20and%20debugging/01_firmware_flashing_instructions.md)

1. Connect the pulse oximeter via UART bridge to your PC as shown [here](https://github.com/IRNAS/pulseox-hardware/blob/v2.x/testing%20and%20debugging/03_debugging.md).

2. Mount the finger clip of your pulse oximeter onto the simulator 

3. Set the SpO2 level on the simulator to the highest possible and run it

4. Begin with data acquisition like it's shown in the [debugging instructions](https://github.com/IRNAS/pulseox-hardware/blob/v2.x/testing%20and%20debugging/03_debugging.md). The 'ratio' value from the acquired data series (second to last value) has a matching SpO2 value, programmed in the [look-up table](https://github.com/IRNAS/pulseox-firmware/blob/master/src/spo2.h). This table has to be adjsuted such that the SpO2 values displayed by GliaX pulse oximeter match the SpO2 values on the simulator at all times. If the SpO2 value displayed on your Glia pulse oximeter is different to the one set on the simulator at any point, replace the current value in the table in  with the one from the simulator.

5. Finish the mapping procedure by reducing the SpO2 level on the simulator in the steps of 0.2% (no need to stick to this value - make the step as small as the simulator can offer) and repeat the procedure down to 85% SpO2 (also TBC, go as low as you can on the simulator, as long as it is sensible).

6. Flash the new clean firmware with the updated lookup table to the device and mount the finger clip back onto the simulator. Now the simulator SpO2 values and the SpO2 values on the display should match.
