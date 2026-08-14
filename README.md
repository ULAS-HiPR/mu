# Mu

Mu is a standalone SiPM/scintillator cosmic-ray detector and flight logger developed alongside the [Ogma avionics stack](https://sean-osullivan.com/projects/ogma/). It is designed to fly as its own black box: battery power in, particle and pressure data logged to onboard flash, then post-flight readout.

> [!CAUTION]
> **Rev 1 assembly corrections:** The `MICROFC-60035-SMT-TR1` SiPM footprint nets are reversed. On an existing Rev 1 PCB, mount U3 180 degrees from the orientation indicated by the footprint's pin-1 marker. This puts SiPM pin 3 (cathode) on PCB pad 1, the filtered HV-bias node, and SiPM pin 1 (anode) on PCB pad 3, the U5 inverting-input node. Pins 2 (fast output) and 4 (no connect) then exchange two unused PCB pads.
>
> The Rev 1 footprint also connects centre pad 5 to AGND. [Onsemi specifies pin 5 as no connect and recommends leaving the paddle unsoldered and floating](https://www.onsemi.com/download/data-sheet/pdf/microc-series-d.pdf). On the working board, a tiny piece of Blu Tack on the PCB beneath pad 5 kept the centre paddle off the board, and the sensor was attached to four small solder balls around the outer pads. A new PCB revision should correct the pin 1/pin 3 net mapping and leave pad 5 floating instead of relying on either assembly workaround.

## Hardware

- 50 x 50 x 20 mm BC-408 plastic scintillator.
- Onsemi MicroFC-60035 6 x 6 mm SiPM biased at about 30 V.
- OPA656/OPA814-class transimpedance readout around a 2.5 V reference.
- STM32F042 controller, W25Q128 external flash, and MS5607 barometer.
- Optional CAN interface; standalone logging does not depend on the Ogma stack.

## Repository

- `mu.kicad_sch`, `power.kicad_sch`, and `mu.kicad_pcb` - KiCad hardware design.
- `fabrication/` - Rev 1 manufacturing outputs.
- `Lib/` - project symbols, footprints, and 3D models.
- `SiPM Test Samples/` - raw oscilloscope samples.

Firmware, the field dashboard, and current flight preparation are on [`mu/f042-bringup`](https://github.com/ULAS-HiPR/mu/tree/mu/f042-bringup).

## Status

Mu has detected and logged real particle pulses on the bench and flew at Mach26, but it did not record a clean in-flight particle run. The reproduced fault is mechanical over-compression of the scintillator/SiPM stack. Packaging and flight validation remain in progress; Mu is not yet flight-proven.

## Manufacturing support

Rev 1 PCB fabrication was sponsored through [EasyEDA Education](https://easyeda.com/education) and manufactured by [JLCPCB](https://jlcpcb.com/). The board was designed in KiCad and imported into EasyEDA Pro for the sponsorship and manufacturing workflow.

## More information

See the [Mu project write-up](https://sean-osullivan.com/projects/mu/) for the detector, bring-up failures, pulse captures, and flight diagnosis.

![Mu schematic](mu-sch.png)
