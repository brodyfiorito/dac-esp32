# Lissajous Figure Generator
A compact, standalone signal generator that produces phase- and frequency-controlled stereo sine waves for Lissajous figure visualization on an oscilloscope. Built around an ESP32-S3 and a PCM5102A stereo DAC, the device outputs two independent audio-frequency signals with real-time control over frequency ratio and phase offset via onboard potentiometers. It also functions as a USB-C audio speaker, with the ESP32-S3 acting as a USB audio device for music visualization on the oscilloscope.

## Overview
When two sine waves are fed into the X and Y inputs of an oscilloscope, the resulting trace is a Lissajous figure — a pattern whose shape is determined entirely by the frequency ratio and phase offset between the two signals. A 1:1 ratio at 90° produces a circle. A 2:1 ratio produces a figure eight. Higher integer ratios yield increasingly complex looping patterns.
<img width="800" alt="Lissjous figure examples" src="https://github.com/user-attachments/assets/c104f9fc-5c8d-4782-b349-47ff7a28968e" />


It's a classic technique for visualizing the relationship between two signals, and the motivation behind this project. The device generates both signals digitally using an ESP32-S3 and outputs them through a PCM5102A stereo DAC, producing clean analog sine waves on two independent channels. Frequency ratio and phase offset are controlled via onboard potentiometers, making it a fully self-contained Lissajous generator.

### Key Features:
* Dual-channel sine wave output via I²S audio DAC
* Real-time frequency ratio and phase offset control
* FreeRTOS-based firmware with state machine architecture
* USB-C audio input mode for music visualization.
* Custom 2-layer PCB design in Altium Designer (in progress)

## Hardware
### System Architecture
| Block | Component | Notes |
| -------- | -------- | -------- |
| Microcontroller   | ESP32-S3-WROOM-1-N4    |Dual-core, handles I²S output and control logic. Opens Bluetooth connectivity options       |
| DAC   | PCM5102A    | 32-bit stereo audio DAC, I²S input      |
| Power   | TLV75733PDYDR    | 1A LDO regulator; ESP32-S3 and PCM5102A draw well under 500mA combined         |
| Input   | 2x USB-C    | One port used for programming the MCU, other used as a speaker         |
| Output   | 4x PCB Headers + JST male header   | Left and right DAC output signals, plus two dedicated ground headers for oscilloscope probe referencing. OLED display port         |

### Design Decisions

**LDO over buck converter:** The TLV75733PDYDR was chosen over a switching regulator specifically for noise performance. A buck converter operating at hundreds of kHz would inject switching noise onto the supply rail, which would appear directly on the PCM5102A analog output. An LDO produces a clean, low-noise 3.3V rail critical for signal integrity on an audio DAC.

**PCM5102A:** Selected for its stereo output and 32-bit resolution, providing more than enough dynamic range for clean sine wave generation. The internal charge pump also eliminates the need for an external negative supply rail, simplifying the power architecture.

**Output RC filter (1kΩ + 15nF):** The DAC output headers include a first-order low-pass filter with a cutoff frequency of ~10.6kHz. This was chosen to pass the full operating frequency range of 210–840Hz with minimal phase shift, while attenuating high-frequency noise and DAC imaging artifacts above the audio band. At 840Hz the phase shift introduced by this filter is under 5°, keeping both channels well-matched for accurate Lissajous figures.

**TVS diode on 5V input:** Protects the power rail from voltage transients on the USB input. USB ports can see inductive spikes during hot-plug events; the TVS clamps these before they reach the LDO or any downstream components.

**Color-coded test points:** Output headers use red (YRIGHT), white (XLEFT), and black (ground) test points to match standard oscilloscope probe color conventions, making it easy to connect the scope without referencing the schematic.

### Pin-Mapping
| Signal | GPIO | Notes |
| -------- | -------- | -------- |
| OLED SDA | GPIO 1 | I²C data — OLED display |
| OLED SCL | GPIO 2 | I²C clock — OLED display |
| PHASE | GPIO 4 | Phase pot — ADC Channel 3 |
| FREQ | GPIO 5 | Frequency pot — ADC Channel 4 |
| MODE SWITCH | GPIO 6 | Mode selection switch |
| LRCK | GPIO 35 | I²S left/right clock |
| DOUT | GPIO 36 | I²S data out to DAC |
| BCK | GPIO 37 | I²S bit clock |
| SCK | GPIO 38 | I²S system clock (MCLK) |


### Schematics
All schematics were captured in Altium Designer. PCB layout is currently in progress.

<img width="800" alt="Sheet 1" src="https://github.com/user-attachments/assets/6a980bcc-2508-4cd6-8966-80163abe863d" />

_Sheet 1 — Input/Output: Dual USB-C connectors (speaker and programming), CP2102-GMR USB-to-UART bridge for MCU programming, phase difference and frequency potentiometer inputs with R-C filtering, left and right DAC output headers with 1k series resistors and 15nF shunt capacitors, and USB D+/D− data lines with 22Ω series resistors._


<img width="800" alt="Sheet 2" src="https://github.com/user-attachments/assets/40fb2dea-fd27-4487-9d91-7cf5b3c41dde" />

_Sheet 2 — Power: 5V to 3.3V regulation via TLV75733PDYDR LDO. Input is protected by a TVS diode with 1µF and 0.1µF decoupling capacitors. The EN pin is controlled by a 2.2kΩ/5kΩ resistor divider tied to the XSMT soft-mute signal. Output is decoupled with a 10µF capacitor, and a power indicator LED is driven through a 680Ω current-limiting resistor._


<img width="800" alt="Sheet 3" src="https://github.com/user-attachments/assets/63065a3a-e260-4ad4-8e4a-344aa9e1ba69" />

_Sheet 3 — Audio: PCM5102A stereo DAC receiving I²S signals (SCK, BCK, DOUT, LRCK) from the ESP32-S3. AVDD, CPVDD, and DVDD are all supplied from 3.3V with 10µF and 0.1µF decoupling. Internal charge pump is supported by 2.2µF capacitors on CAPP and CAPM. Left (OUTL) and right (OUTR) analog outputs are routed to the output header section. XSMT soft-mute is tied to the power section EN divider._


<img width="800" alt="Sheet 4" src="https://github.com/user-attachments/assets/52cfc1fc-52ae-44ad-b07f-2947303b3823" />

_Sheet 4 — MCU: ESP32-S3-WROOM-1-N4 with 10µF and 0.1µF decoupling on the 3.3V rail. I²S signals (LRCK, DOUT, BCK, SCK) are routed to the PCM5102A. UART RX/TX connect to the CP2102-GMR programming bridge via RTS/DTR auto-reset circuit — RTS and DTR signals drive NPN transistors through base resistors to control the EN and IO0 pins for automatic bootloader entry. USB D+/D− are broken out for USB audio (speaker) mode. Analog inputs receive the phase difference and frequency potentiometer signals. A mode selection switch with pull-down resistor and debounce capacitor is included. I²C lines (SCL, SDA) are broken out for the optional OLED display with decoupling on the supply line._


### Bill of Materials
| Ref | Description | Value / Part | Package | Qty |
|-----|-------------|--------------|---------|-----|
| MCU | WiFi/BT Module | ESP32-S3-WROOM-1-N4 | SMD Module | 1 |
| DAC | Stereo DAC | PCM5102APWR | 20-TSSOP | 1 |
| PROG | USB-to-UART Bridge | CP2102-GMR | 28-QFN | 1 |
| U1 | LDO Regulator | TLV75733PDYDR | SOT-23-5 | 1 |
| J1, J2 | USB-C Connector | 2012670005 | SMD RA | 2 |
| P1 | OLED Header | JST 4-Pin 2mm | 4-Pin THD | 1 |
| Q1, Q2 | NPN Transistor | SOT-1123 | SOT-1123 | 2 |
| R12 | Phase Potentiometer | ALPS RK11K1140A3L | 4-Pin THD | 1 |
| R13 | Frequency Potentiometer | ALPS RK11K1140A3L | 4-Pin THD | 1 |
| R1–R4, R15 | Resistor | 5kΩ | 0603 | 5 |
| R6, R7, R10, R11 | Resistor | 1kΩ 1% 0.1W | 0603 | 4 |
| R8, R9 | Resistor | 22Ω 5% 0.1W | 0603 | 2 |
| R14 | Resistor | 2.2kΩ 1% 0.1W | 0603 | 1 |
| R16 | Resistor | 680Ω 5% | 0603 | 1 |
| R17–R21 | Resistor | 10kΩ 0.1% | 0603 | 5 |
| C1, C7, C19 | Capacitor | 1µF | 0603 | 3 |
| C2, C3, C4, C8, C10, C11, C14, C15, C17 | Capacitor | 0.1µF X7R | 0603 | 9 |
| C5, C6 | Capacitor | 15nF X7R | 0603 | 2 |
| C9, C16, C18 | Capacitor | 10µF X5R | 0603 | 3 |
| C12, C13 | Capacitor | 2.2µF X7S | 0603 | 2 |
| D1 | TVS Diode | 5V/14V | 2x2 SON | 1 |
| D2 | LED | Red Clear | 2-SMD | 1 |
| SW1 | Slide Switch | SPST 4A 125V | THD | 1 |
| SGND1, SGND2 | Test Point | Black Miniature | — | 2 |
| XLEFT | Test Point | White Miniature | — | 1 |
| YRIGHT | Test Point | Red Miniature | — | 1 |
## Controls

The device has three physical controls: two potentiometers and a mode slide switch.

| Control | Function |
|---|---|
| Frequency pot | Selects frequency ratio between channels |
| Phase pot | Selects phase offset between channels |
| Mode switch | Toggles between Lissajous generator and USB audio input modes |

### Frequency Ratios
| Pot Position | Ratio | Pattern |
|---|---|---|
| 1 | 1:1 | Ellipse / circle |
| 2 | 2:1 | Figure eight |
| 3 | 3:1 | Three-lobed curve |
| 4 | 4:1 | Four-lobed curve |
| 5 | 1:2 | Horizontal figure eight |
| 6 | 3:2 | Three/two curve |
| 7 | 4:3 | Four/three curve |
| 8 | 6:5 | Six/five curve |

### Phase Steps
| Pot Position | Phase Offset |
|---|---|
| 1 | 0 |
| 2 | π/8 |
| 3 | π/4 |
| 4 | 3π/8 |
| 5 | π/2 |
| 6 | 5π/8 |
| 7 | 3π/4 |
| 8 | 7π/8 |

### Modes
**Lissajous Mode** — The ESP32-S3 generates both sine wave channels internally. Frequency ratio and phase offset are controlled via the onboard potentiometers. Connect XLEFT and YRIGHT outputs to the X and Y inputs of an oscilloscope in XY mode.

**Audio Input Mode** — The ESP32-S3 enumerates as a USB audio device. Audio played from a connected computer is passed through to the DAC outputs, allowing music to be visualized on the oscilloscope in XY mode.

## Firmware

The firmware is written in Embedded C using the ESP-IDF framework with FreeRTOS. Sine wave generation uses a phase accumulator, rather than computing sine values from absolute time, each sample increments a floating-point phase register by a step size proportional to the desired frequency, which avoids unbounded growth of the time variable.

### Architecture Highlights:
* FreeRTOS task handles I²S DMA buffer filling at audio sample rate
* State machine manages mode switching
* Phase accumulator for drift-free, long-term stability
* Exponential moving average + hysteresis on ADC pot readings to prevent jitter and snap between ratio/phase steps cleanly

Built with ESP-IDF v5.5.2

## Development
### Devkit Prototype
Initial development and firmware validation were done on an ESP32-S3 devkit connected to a PCM5102A breakout board. During testing, the left channel output of the PCM5102A breakout was found to be non-functional. The right channel operated correctly. To rule out a firmware bug, the left and right channels were swapped in software, confirming the issue was a hardware fault on the breakout module rather than anything in the code. A bench signal generator was substituted as the left channel input to the oscilloscope, allowing firmware validation and Lissajous figure generation to continue. Outside of the broken channel, the devkit setup was used to fully validate the signal generation algorithm, I²S configuration, and oscilloscope output before committing to PCB layout.

![ESP32 on a breadboard](https://github.com/user-attachments/assets/b15572c4-cad9-44af-8b62-75a6d9feb1a6)
_Breadboard prototype — ESP32-S3 devkit connected to PCM5102A breakout via jumper wires. Oscilloscope probe attached to OUTR and ground. Phase and frequency potentiometers wired in via jumper wires._

![Two sine waves](https://github.com/user-attachments/assets/2ba17c26-6805-43bd-a2a0-bff6a696a753)
_Rigol DS1102E oscilloscope showing early firmware output. CH1 (yellow) is the signal generator left channel reference, CH2 (blue) is OUTR from the PCM5102A; both set to 210Hz. The frequency readout is being thrown off by noise on the signals._

<video src="https://github.com/user-attachments/assets/7f4eb666-647e-48f4-968a-9c4041df8248" controls width="600"></video>
_Rigol DS1102E in XY mode. 210Hz on both channels with a phase offset producing a rotating ellipse pattern. CH1 (signal generator) on the X axis, CH2 (PCM5102A OUTR) on the Y axis._

### PCB Design
The schematic is complete in Altium Designer and PCB routing is currently underway. The design integrates the ESP32-S3, PCM5102A DAC, TLV75733PDYDR LDO, probe outputs, and USB-C connectors onto a single board.

## Roadmap
- [ ] Complete PCB layout in Altium Designer
- [ ] Fabricate and assemble first PCB revision
- [ ] Validate both DAC output channels on hardware
- [x] Explore OLED display for parameter readout
  - [x] Add Hardware to Schematic
  - [ ] Program Display
- [ ] Enclosure design
