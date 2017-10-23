# Calibration method

Use the patient simulator that enables pulse oximetry simulation. 

0. Flash the calibration firmware (calibration_firmware.bin) onto your Glia pulse oximeter as shown in [these instructions](https://github.com/IRNAS/pulseox-hardware/blob/v2.x/testing%20and%20debugging/01_firmware_flashing_instructions.md)

1. Connect the pulse oximeter via UART bridge to your PC as shown [here](https://github.com/IRNAS/pulseox-hardware/blob/v2.x/testing%20and%20debugging/03_debugging.md).

2. Mount the finger clip of your pulse oximeter on to the simulator 

3. Set the SpO2 level on the simulator to the highest possible and run it

4. Begin with data acquisition like it's shown in the [debugging instructions](https://github.com/IRNAS/pulseox-hardware/blob/v2.x/testing%20and%20debugging/03_debugging.md). The acquired raw value that you can see on the from your Glia pulse oximeter is correspondent to the respective SpO2 level on the simulator. Replace it in the [deault look up table](https://github.com/IRNAS/pulseox-firmware/blob/master/src/spo2.h). 

5. Finish the mapping procedure by reducing the SpO2 level on the simulator by in the steps of 0.2% (TBC) and repeat the procedure down to 90% SpO2.

6. Flash the new clean firmware to the device and mount the finger clip back on tho the simulator. Now the simulator values and the values on the display should match.
