# Amphibious Tensegrity Spherical Robot

> An open-source STM32 firmware project for a six-strut amphibious tensegrity spherical robot. 

本项目面向湿地、浅水区及复杂水陆过渡环境的生态勘测需求，开发一种具有多模态运动能力的水陆两栖张拉整体球形机器人。机器人采用六支柱张拉整体结构与气动驱动系统，可实现陆地滚动与跳跃、水下运动以及水陆过渡区域运动。

This project develops a six-strut amphibious tensegrity spherical robot for ecological surveying in wetlands, shallow water, and complex land–water transition zones. The robot combines a tensegrity structure with pneumatic actuation to support terrestrial rolling and jumping, underwater locomotion, and cross-medium movement.

---

## Robot Overview

<p align="center"> 
  <img src="media/robo.jpg"
       alt="Amphibious tensegrity spherical robot"
       width="750">
</p> 

---

## Contents

- [Project Overview](#project-overview)
- [Authors and Contributors](#authors-and-contributors)
- [Main Features](#main-features)
- [System Architecture](#system-architecture)
- [Hardware](#hardware)
- [Software and Control](#software-and-control)
- [Repository Structure](#repository-structure)
- [Development Environment](#development-environment)
- [Getting Started](#getting-started)
- [Operation Procedure](#operation-procedure)
- [Safety Notice](#safety-notice)
- [Project Status](#project-status)
- [Contributing](#contributing)
- [License](#license)
- [Citation](#citation)
- [Acknowledgements](#acknowledgements)
- [Contact](#contact)

---

## Project Overview

Traditional rigid robots often face difficulties in wetlands and land–water transition zones, including poor terrain adaptability, limited impact resistance, and possible disturbance to fragile environments. A tensegrity robot uses isolated compression members connected by a continuous tension network, giving the overall structure flexibility, impact resistance, and passive environmental adaptability.                

The current platform is based on a six-strut tensegrity spherical structure. Its embedded controller drives multiple pneumatic valves in a predefined or feedback-adjusted sequence, changing cable tension and the robot's center of mass to generate different gaits.              

The project currently focuses on:

- STM32-based embedded control;
- 12-channel pneumatic valve actuation;
- gait-sequence execution and switching;
- ELRS remote-control input;
- attitude and motion-state acquisition;
- land, underwater, and transition-zone locomotion;
- system integration, debugging, and experimental verification.

---

## Authors and Contributors

The original firmware was primarily developed by **RuoYv Ren**, who completed the initial implementation of the robot's basic control program and locomotion functions.     

**Angelia Ye** subsequently improved and optimized the original program, completed system integration, debugging, functional verification, and code organization, and prepared the source code and documentation for open-source release.      

This repository is published with the permission of the original firmware author.      

### Contribution Statement

| Contributor | Main contribution |
|---|---|
| **RuoYv Ren** | Original firmware development; initial implementation of basic robot control and locomotion functions; debugging; functional verification |     
| **Angelia Ye** | Program improvement and optimization; system integration; debugging; functional verification; documentation; repository organization and maintenance |     


---

## Main Features

- Six-strut tensegrity spherical structure;
- pneumatic actuation through multiple independently controlled solenoid valves;
- 12-channel actuation-sequence control;
- terrestrial rolling and jumping;
- underwater rolling and jumping;
- water–land transition locomotion;
- predefined gait execution and gait switching;
- STM32-based real-time embedded control;
- SBUS/ELRS remote-control interface;
- support for IMU and attitude-sensor integration;
- extensible interface for multi-source sensing and closed-loop control.

---

## System Architecture

The robot control system can be summarized as:

```text
Remote controller / upper-level command
                  │
                  ▼
             ELRS receiver
                  │
                  ▼
            STM32 controller
          ┌───────┴────────┐
          │                │
          ▼                ▼
  Gait and state logic   Sensor acquisition
          │          IMU / attitude sensors
          ▼                │
  12-channel control       │
          └───────┬────────┘
                  ▼
       Pneumatic valve driver
                  │
                  ▼
      Pneumatic actuators / cables
                  │
                  ▼
    Rolling, jumping, and transition motion
```

The current firmware executes the required pneumatic action sequence according to the operating state and remote-control input. Sensor feedback and closed-loop gait adjustment can be extended on top of this framework.

---

## Hardware

The robot control system may include the following hardware:

| Module | Function |
|---|---|
| STM32 controller | Executes the main control program, gait logic, communication, and signal output |
| 12-channel valve driver | Converts controller signals into valve-driving signals |
| 24 V solenoid valves | Control pneumatic flow paths and actuator states |
| Pneumatic actuation system | Produces cable-tension or structural-deformation actions |
| ELRS receiver | Receives remote-control commands |
| IMU / attitude sensors | Measure acceleration, angular velocity, heading, or robot attitude |
| SWD interface | Supports firmware download and online debugging |
| Power-supply modules | Provide isolated or regulated voltages required by the controller and loads |

### Important Electrical Notes

- Confirm the controller logic voltage before connecting the valve-driver board.
- Confirm that the solenoid-valve supply is 24 V and that the driver is suitable for the valve current.
- The controller ground, driver ground, and communication ground must be connected according to the actual circuit design.
- Verify every control channel before connecting the full pneumatic system.
- Do not power the controller or sensors directly from an unregulated 24 V source.
- Check polarity, fuse rating, connector orientation, and insulation before power-on.

---

## Software and Control

The firmware is developed for an STM32 embedded platform and opened or compiled using Keil MDK. 

The program may contain the following functional modules:

- system initialization;
- GPIO and peripheral configuration;
- ELRS data reception;
- remote-control link-state detection;
- channel-value decoding;
- gait state machine;
- pneumatic action scheduling;
- 12-channel valve control;
- timing and periodic control;
- sensor acquisition;
- fault handling and safe shutdown.

### Gait-Control Logic

A typical gait-control process is:

1. Initialize the controller, communication interface, GPIO, timers, and sensors.
2. Check the remote-control link state.
3. Decode the selected channels.
4. Determine whether the robot is enabled and which gait is requested.
5. Execute the corresponding pneumatic action sequence.
6. Update the action counter or gait state periodically.
7. Read sensor information when available.
8. Stop the actuator output when a fault, link loss, or shutdown command is detected.

> The exact channel thresholds, timing parameters, actuator numbers, and action sequences are determined by the source code and the specific robot configuration. Do not operate the physical robot until these values have been checked.

---

## Repository Structure

The repository currently centers on the STM32 firmware. the structure is shown below:

```text
Tensegrity-Amphibious-Robot/
├── README.md
├── LICENSE
├── .gitignore
└── firmware/
    ├── Core/                 # STM32 application source and headers
    ├── DDrivers/             # Original peripheral configuration 
    ├── Drivers/              # STM32 HAL/CMSIS drivers
    ├── Middlewares/          # Optional middleware
    ├── MDK-ARM/              # Keil project configuration
    └── User/                 # Optional user-written control modules
```

The following directories may be added later when the corresponding materials are ready for public release:

```text
hardware/                     # Schematics, PCB files, BOM, and interface definitions
simulation/                   # Gait, control, or mechanical simulation
docs/                         # Technical documentation and experiment instructions
``` 

### Files That Should Usually Be Retained

- `*.c` and `*.h`;
- `*.ioc`;
- `*.uvprojx`;
- `*.uvoptx`;
- required startup files;
- linker scripts;
- necessary drivers and middleware;
- configuration files required to reproduce the build.

### Files That Should Usually Be Ignored

- `Objects/`;
- `Listings/`;
- `Debug/`;
- `Release/`;
- `*.o`;
- `*.d`;
- `*.dep`;
- `*.axf`;
- `*.elf`;
- `*.map`;
- `*.lst`;
- temporary files and local IDE settings;
- passwords, tokens, private keys, and local secret files.

---

## Development Environment

The exact versions used for a reproducible build should be recorded here.          

| Item | Version / model |
|---|---|
| MCU | STM32F405 |
| Keil MDK | Keil uVision5 |
| ARM compiler | ARMCC 5.06 Update 6 (Build 750) |
| STM32CubeMX | STM32CubeMX 6.14.1 |
| Debugger | J-LINK |
| Remote-control link | ELRS receiver with SBUS output |
| Solenoid-valve voltage | 24 V |
| Number of actuation channels | 12 |        

---

## Getting Started

###  1. Clone the Repository

Using Git:

```bash
git clone https://github.com/angelia-2007/Tensegrity-Amphibious-Robot.git
cd Tensegrity-Amphibious-Robot
```

Alternatively, open the repository page, select **Code → Download ZIP**, and extract the downloaded file.   

### 2. Locate the Keil Project

Open the `firmware/` directory and locate the Keil project file:

```text
*.uvprojx
```

Double-click the file or open it from:

```text
Keil MDK → Project → Open Project
```

### 3. Check the Target Device

In Keil MDK:

1. Open **Project → Options for Target**.
2. Confirm that the selected STM32 model matches the physical controller.  
3. Confirm the oscillator and clock configuration.
4. Confirm the compiler version.
5. Confirm the include paths and preprocessor definitions.
6. Confirm that all source files are present and not marked as missing.

### 4. Compile the Firmware

Click:

```text
Project → Build Target
```

or press:

```text
F7
```

A successful build should report zero errors. Warnings should also be checked rather than ignored automatically.

### 5. Connect the Debugger

1. Disconnect the 24 V actuator power supply.
2. Connect the SWD debugger to the controller.
3. Confirm the SWDIO, SWCLK, GND, and reference-voltage connections.
4. In **Options for Target → Debug**, select the correct debugger.
5. Open **Settings** and confirm that the target device can be detected.

### 6. Flash the Firmware

After a successful build:

1. Keep the pneumatic load in a safe, disabled state.
2. Click **Download** in Keil.
3. Wait for the programming and verification process to complete.
4. Reset the controller.
5. Verify the logic-level output before connecting the valves.

---

## Operation Procedure

The following sequence is recommended for the first physical test.

### Stage 1: Source-Code Inspection

- Confirm the MCU model and pin definitions.
- Confirm the control polarity of all valve channels.
- Confirm whether a logic high or logic low activates each output.
- Confirm the SBUS channel used as the enable switch.
- Confirm channel thresholds and link-loss behavior.
- Confirm the gait timing parameters.
- Confirm the maximum number of simultaneously activated valves.

### Stage 2: Controller-Only Test

- Keep all solenoid valves disconnected.
- Power only the low-voltage controller.
- Verify that the program starts normally.
- Verify the remote-control link.
- Observe the relevant GPIO outputs using an LED, logic analyzer, or oscilloscope.
- Confirm that disabling the robot turns off every actuator command.

### Stage 3: Single-Channel Valve Test

- Connect only one valve channel.
- Use a current-limited power supply if available.
- Confirm the driver output and valve action.
- Check the MOSFET, driver, connector, and valve temperature.
- Repeat the test for all channels.

### Stage 4: Multi-Channel Pneumatic Test

- Set the regulator to a conservative pressure.
- Secure the robot to prevent uncontrolled movement.
- Test one gait at low speed.
- Confirm that the actual actuator numbering matches the program.
- Check tubes, fittings, valves, cables, and structural members.

### Stage 5: Free-Motion Test

- Move the robot to a controlled test area.
- Keep people away from the motion path.
- Prepare an emergency power cutoff.
- Start with terrestrial low-speed motion.
- Test jumping, underwater movement, and transition-zone movement only after the basic rolling gait is reliable.

---

## Safety Notice

This robot contains pneumatic components, moving structural members, tensioned cables, and 24 V electrical loads. Improper operation may cause unexpected movement, cable release, air leakage, component overheating, or mechanical impact.

Before every test:

- verify the rated pressure of all pneumatic components;
- inspect tubes, fittings, cables, and structural members;
- confirm the power-supply polarity and voltage;
- confirm that an overcurrent-protection device is installed;
- keep the robot disabled during wiring;
- provide a physical emergency power cutoff;
- keep hands and loose objects away from moving or tensioned parts;
- keep personnel away from the predicted rolling or jumping path;
- stop immediately if abnormal heat, smell, noise, leakage, or unstable motion is observed.

Do not exceed the rated pressure, voltage, current, or mechanical load of any component.

This repository is intended for research and educational use. Users are responsible for validating the design, wiring, firmware, and operating procedure for their own hardware.

---

## Project Status

The repository is under active organization and development.

| Item | Status |
|---|---|
| Original STM32 firmware | Available / being organized |
| Program optimization and debugging | Completed for the current prototype |
| Functional verification | Completed for the current prototype configuration |
| Keil project documentation | In progress |
| Hardware schematics and PCB files | Not yet publicly released |
| Simulation files | Not yet publicly released |
| Reproducible build-version record | To be completed |
| Demonstration media | To be added |

The code should be regarded as research prototype software rather than a production-ready control system.

---

## Contributing

Issues and improvement suggestions are welcome.

Before contributing:

1. Open an issue describing the problem or proposed feature.
2. Create a separate branch for the modification.
3. Keep changes focused and document the reason for each change.
4. Test the firmware on suitable hardware or clearly state that it has not been physically tested.
5. Do not commit passwords, tokens, personal information, confidential documents, or unauthorized third-party code.
6. Submit a pull request with a clear description of the modification and validation method.

Suggested commit messages:

```text
Add valve output safety reset
Improve gait switching logic
Update STM32 pin configuration
Document pneumatic channel mapping
```

---

## License

The source code in this repository is distributed under the license specified in the [`LICENSE`](LICENSE) file.

Before adding a license, all relevant code contributors and rights holders has confirmed that:

- they agree to the public release of their contributions;
- they agree to the selected open-source license;
- the repository does not contain confidential or unauthorized third-party material;
- the release does not conflict with an unpublished patent application, competition rule, university requirement, or collaboration agreement.

---

## Citation

If this repository is useful in your research, project, or publication, please cite it as:

```text
Ren, R., and Ye, A. Amphibious Tensegrity Spherical Robot.
GitHub repository, 2026.
https://github.com/angelia-2007/Tensegrity-Amphibious-Robot
```

BibTeX:

```bibtex
@software{ren_ye_2026_tensegrity_robot,
  author  = {Ren, RuoYv and Ye, Angelia},
  title   = {Amphibious Tensegrity Spherical Robot},
  year    = {2026},
  url     = {https://github.com/angelia-2007/Tensegrity-Amphibious-Robot},
  note    = {GitHub repository}
}
```

If a formal paper, DOI, or archived release becomes available, we will replace this temporary repository citation with the formal citation.

---

## Acknowledgements

We sincerely acknowledge the team members, instructors, and collaborators who contributed to the mechanical design, embedded control, pneumatic-system integration, system testing, and experimental verification of the robot.

Any person listed by name in future versions of this section will be informed and should agree to the public attribution.

---

## Contact

For technical questions, bug reports, or collaboration proposals, please use the repository's **Issues** page:

```text
https://github.com/angelia-2007/Tensegrity-Amphibious-Robot/issues
```

To protect personal privacy, avoid placing private phone numbers, personal addresses, access tokens, or other sensitive information in this public README.
