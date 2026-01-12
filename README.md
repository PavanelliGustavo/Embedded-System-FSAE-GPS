# FSAE GPS Telemetry System 🏎️💨

![Project Status](https://img.shields.io/badge/Status-In%20Development-yellow)
![Platform](https://img.shields.io/badge/Platform-MSP430-red)
![Language](https://img.shields.io/badge/Language-C%20%7C%20Python-blue)

**High-precision, low-cost GPS telemetry and timing system developed for the Apuama Racing Formula SAE Team (University of Brasília).**

## 📖 Overview

The Apuama Racing team identified a critical gap in data acquisition during dynamic testing. Commercial telemetry solutions are often expensive "black boxes" that are difficult to integrate with custom vehicle networks.

This embedded system is designed to provide:
1.  **Autonomous Timing:** Specifically designed for the FSAE *Acceleration* event (0-75m), eliminating manual stopwatch errors.
2.  **Trajectory Logging:** Records high-frequency GNSS data to plot the vehicle's racing line for driver feedback.
3.  **Future Integration:** Planned support for CAN Bus communication with the ECU (FuelTech) and other vehicle nodes.

## 📂 Repository Structure

```text
FSAE-GPS-Telemetry/
│
├── docs/                           # Technical documentation and manuals
│   ├── datasheets/                 # Component PDFs (MSP430, G10A, LM2596)
│   ├── images/                     # Images for README and documentation
│   └── project_documentation.pdf   # Full technical specification document
│
├── firmware/                       # MSP430 Source Code (C/C++)
│   ├── include/                    # Header files (.h)
│   ├── prototypes/                 # Isolated functional prototypes
│   │   ├── 01_raw_nmea_read/       # Raw NMEA data stream test
│   │   ├── 02_gps_coord_filter/    # Latitude/Longitude parser
│   │   ├── 03_acceleration_timer/  # Acceleration event timer (0-75m)
│   │   └── 04_gps_track_stream/    # Plotter of the route on the map
│   └── src/                        # Unified main firmware (Planned)
│
├── hardware/                       # Electronic Design
│   ├── bom.csv                     # Bill of Materials
│   ├── wiring_diagram.png          # System connection diagram
│   └── cad_models/                 # 3D printable case files (.stl)
│
├── tools/                          # Host-side Analysis Tools
│   └── track_plotter/              # Python visualization script
│       ├── plotter.py              # Real-time path plotting logic
│       └── requirements.txt        # Python dependencies (matplotlib, pyserial)
│
├── .gitignore                      # Build artifacts and local config exclusions
├── LICENSE                         # MIT License
└── README.md                       # Project overview and documentation
```

## 🛠️ Hardware Architecture

The system is built around reliability and modularity to withstand the vibration and electrical noise of a combustion racing prototype.

| Component | Model | Function | Engineering Reasoning |
| :--- | :--- | :--- | :--- |
| **MCU** | TI MSP-EXP430F5529LP | Core Processing | Selected for low power consumption, robust UART/SPI peripherals, and team familiarity from academic coursework. |
| **GNSS** | Quescan G10A-F30 (U-blox M10) | Positioning | Chosen for its high update rate (25Hz)—essential for capturing high-speed vehicle dynamics—and ease of UART integration. |
| **Power** | LM2596 Buck Converter | Voltage Regulation | A switching regulator (Buck) was chosen over linear regulators to efficiently step down the noisy 12V battery voltage to 3.3V with minimal heat dissipation. |
| **Storage** | MicroSD Module | Data Logging | SPI interface for local data retention during runs. |

### Wiring & Pinout
*For a complete schematic, please refer to `docs/datasheets/`.*

The GNSS module communicates via UART with the MSP430:

* **GPS TX** $\rightarrow$ **P3.4 (UCA0RXD)**
* **GPS RX** $\rightarrow$ **P3.3 (UCA0TXD)**
* **VCC/EN** $\rightarrow$ **3.3V**
* **GND** $\rightarrow$ **GND**

## 💻 Firmware & Software Structure

### Firmware (MSP430 / C)
The embedded code is currently modularized into functional prototypes for testing and validation:

* `firmware/prototypes/03_accel_timer/`: **Acceleration Event Logic**. Detects movement start (Speed $\ge$ 1.5 km/h) and automatically stops the timer after 75 meters are traversed.
* `firmware/prototypes/02_gps_coord_filter/`: **NMEA Parser**. Filters `$GNGGA` and `$GNRMC` sentences to extract raw Latitude and Longitude data.
* `firmware/prototypes/01_raw_nmea_read/`: **UART Buffer**. Basic reliable reading of the GNSS stream.

### Analysis Tools (Python)
Located in `tools/track_plotter/`.
* **Track Plotter:** A Python script that processes the collected coordinate logs to visualize the track path and racing line.

## 🚀 Roadmap

* [ ] **Full SD Card Logging:** Finalize SPI implementation for long-duration logging (Enduro event).
* [ ] **Odometer:** Implement cumulative distance tracking for chassis maintenance schedules.
* [ ] **Unified Firmware:** Merge current prototypes into a single selectable menu system.
* [ ] **CAN Bus:** Integrate with FuelTech ECU for data correlation.

## 🤝 Contribution & Team

This project is part of the electrical subsystem development for the 2025/2026 season vehicle.

* **Developer:** Gustavo Pavanelli
* **Team:** Apuama Racing Formula SAE - University of Brasília (UnB).
* **Location:** Brasília, DF, Brazil.

---
*For more information, please contact the developer or open an issue in this repository.*