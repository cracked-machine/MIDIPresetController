# MIDIPresetController

This is a simple MIDI controller for Guitar FX pedals. An [STM32L011D4](HW/techdocs/stm32l011d4_datasheet.pdf) is used to detect stomp switch presses by the user and output MIDI signals via a DIN5 connector.

The project is designed to fit into a [Hammond 1455J1602](HW/techdocs/Hammond_1455J1602BK_enclosure.pdf) enclosure. 

3D models can be found here for the [end plate bezel](https://3dviewer.net#model=https://raw.githubusercontent.com/cracked-machine/MIDIPresetController/master/HW/res/3d/1455J1602/J%20Bezel%20Closed.igs) and [main extrusion](https://3dviewer.net#model=https://raw.githubusercontent.com/cracked-machine/MIDIPresetController/master/HW/res/3d/1455J1602/J%20Extrusion%20160mm.igs)

This is an extruded alu case. The PCB slides in from one side.  

The stomp switches are panel mounted on the front side of the enclosure with connecting wires to rear of the PCB (via molex KK254 connectors)

![](https://lucid.app/publicSegments/view/d0263067-ba71-4093-95e3-58d78a0ea74f/image.png)



![](https://lucid.app/publicSegments/view/eafe9af3-4616-4821-b5d2-7ef66f2ba69d/image.png)

It is powered by a 3v3 CR2032 cell, held by a battery clip, which is mounted on the rear side of the PCB. Internal voltage reference of the MCU is used to measure remaining battery. An LED mounted on the front of the PCB and connected to GPIO indicates low battery power to the user.

The MIDI channel can be selected using the DIP switch on the rear side of the PCB.  The MIDI output connector (DIN5) is mounted to the end plate bezel of the enclosure

A 10-pin ARM SWD programming connector is mounted to the rear of the PCB. 

A removable "belly plate" on the enclosure provides access to the battery, MIDI DIP switch and SWD connector.  


### Development Environment

YOu can run this project from within a container and program the board via a Segger JLink. This uses `JLinkRemoteServer` to establish a connect between the container environment and the device connected to the `host` PC. To enable this connection, run the follow command on the __host__ terminal:

```
JLinkRemoteServer --select USB
```

See [stm32_dev_docker/README.md](https://github.com/cracked-machine/stm32_dev_docker) for instructions on using the `development environment` for this project.

Note: this project is currently configured for a specific version of CMake and ARM compiler. The paths are hard-coded in the project [here](.vscode/settings.json)


### RevB Todo

- `R6` and `R5` pull-down resistors should be pull-up resistors to counter-act `J4` and `J5` switches that are connected to `ground`. 
    - Workaround: DNF R6 and R5.
- Move `J3` footprint away from NRST `SW1` footprint. 
    - Workaround: It is still possible to plug debug cable into `J3` without it being obstructed by `SW1`.

### SW TODO

- Enable timer for debounce
- Enable USART output
- Encode MIDI mesg in USART
- Enable VREF for battery level monitoring