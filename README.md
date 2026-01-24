# Hexapod Robot Firmware

> **A six-legged walking robot firmware featuring adaptive terrain following, active stabilization, and a modular architecture.**


---

## Table of Contents

- [About the Project](#about-the-project)
- [Key Features](#key-features)
- [System Architecture](#system-architecture)
- [Hardware Requirements](#hardware-requirements)
- [Documentation](#documentation)
- [Project Status](#project-status)
- [Contributors](#contributors)

---

## About the Project

### The Rebuild Philosophy

This firmware represents a **complete architectural rewrite** following professional embedded systems principles:

**Layered Architecture**: Clean separation between hardware drivers (BSP), core algorithms (middleware), and high-level behavior (application). This makes the system:
- **Modular**: Swap hardware without touching control algorithms
- **Testable**: Validate algorithms in simulation before hardware deployment
- **Maintainable**: Debug one layer at a time
- **Extensible**: Add new features without breaking existing code

**Model-Based Design**: All control algorithms were first validated in MATLAB/Simulink using the *exact same C code* that runs on the STM32. This caught bugs early and reduced hardware integration time by 70%.

**Real Documentation**: Every module includes design rationale, mathematical derivations, and worked examples. Students can learn *why* decisions were made, not just *what* the code does.

### Why Hexapods Matter

Six-legged robots excel where wheeled or tracked vehicles fail:
- **Terrain Versatility**: Climb stairs, navigate rubble, cross gaps
- **Redundancy**: Continue walking with one leg damaged  
- **Static Stability**: Always maintain 3+ legs on ground (no balancing required)
- **Harsh Environments**: Industrial inspection, search-and-rescue, planetary exploration

### What This Robot Can Do

Today, the system provides:
- ✅ **Multi-gait locomotion**: Tripod (speed), Wave (stability), Ripple (efficiency)
- ✅ **Omnidirectional motion**: Walk/strafe in any direction, rotate in place
- ✅ **Active terrain adaptation**: Real-time ground height detection using contact sensors
- ✅ **PID balance control**: Maintain level body on slopes up to ±30°
- ✅ **Wireless control**: NRF24L01+ (2.4GHz) or iBUS RC receiver
- ✅ **Professional codebase**: Ready for academic projects or commercial adaptation

---

## Key Features

### 🎯 Control Capabilities

**Gait System**:
- **Tripod Gait**: Fast walking (two alternating groups of 3 legs)
- **Wave Gait**: Maximum stability (one leg moves at a time)
- **Ripple Gait**: Balanced speed and efficiency

**Motion Control**:
- Body-relative omnidirectional translation (forward/back, strafe, rotate)
- 6-DOF body pose control (X, Y, Z, Roll, Pitch, Yaw)
- Smooth acceleration profiles (no mechanical shock)

**Terrain Adaptation**:
- Dual-rate contact detection (200Hz fast poll + 50Hz slow read)
- Per-leg height memory (handles rocks, holes, stairs)
- Smooth height blending (no jerky transitions)

### 🧠 Architecture Highlights

**Three-Layer Design**:
```
Application Layer:  Gait Engine, FSM, Behavior Planning
        ↓
Middleware Layer:   IK Solver, Sensor Fusion, PID Controllers
        ↓
Hardware Layer:     Servo PWM, IMU I2C-DMA, Contact Sensors
```

**Real-Time Performance**:
- Hard real-time scheduler (50Hz physics loop, 200Hz sensor loop)
- Zero blocking operations (all I/O uses DMA or interrupts)
- Cooperative scheduling (no RTOS overhead)

**Model-Based Validation**:
- Control algorithms tested in MATLAB/Simulink *before* hardware
- Simscape Multibody physics simulation
- Same C code runs in simulation and on STM32 (verified 1:1 match)

### 🔧 Hardware Abstraction

**18 Servo Channels**: Multi-timer PWM with per-servo calibration  
**6-Axis IMU**: MPU6050 with complementary filter (drift-free orientation)  
**Ground Contact**: 6 microswitches with latch mechanism (never miss brief contacts)  
**Wireless Control**: NRF24L01+ or Flysky iBUS receiver  

---

## System Architecture

The firmware follows strict **layered architecture** to ensure modularity and testability:

```
┌─────────────────────────────────────────────────────────────┐
│                   APPLICATION LAYER                         │
│  ┌──────────────────┐  ┌────────────┐  ┌────────────────┐  │
│  │  Gait Scheduler  │  │  FSM       │  │  RC Input      │  │
│  │  (Phase-Driven)  │  │  (4 Modes) │  │  Processing    │  │
│  └──────────────────┘  └────────────┘  └────────────────┘  │
└─────────────────────────────────────────────────────────────┘
                           ▼
┌─────────────────────────────────────────────────────────────┐
│                   MIDDLEWARE LAYER                          │
│  ┌─────────────┐  ┌──────────────┐  ┌──────────────────┐   │
│  │ IK Solver   │  │ Sensor       │  │ PID Controllers  │   │
│  │ (3-DOF)     │  │ Fusion       │  │ (Pitch/Roll)     │   │
│  └─────────────┘  └──────────────┘  └──────────────────┘   │
└─────────────────────────────────────────────────────────────┘
                           ▼
┌─────────────────────────────────────────────────────────────┐
│            HARDWARE ABSTRACTION LAYER (BSP)                 │
│  ┌─────────┐  ┌─────────┐  ┌────────┐  ┌──────────────┐   │
│  │ IMU     │  │ Servos  │  │ Limits │  │ RC Receiver  │   │
│  │ (I2C)   │  │ (PWM)   │  │ (GPIO) │  │ (SPI/UART)   │   │
│  └─────────┘  └─────────┘  └────────┘  └──────────────┘   │
└─────────────────────────────────────────────────────────────┘
```

### Data Flow

**[SENSING]**  
IMU (I2C DMA) → Sensor Fusion (Complementary Filter) → Orientation (Pitch/Roll)

**[CONTROL]**  
RC Input → FSM → Mode Selection → Gait Parameters  
Orientation → PID Controller → Body Pose Correction  

**[PLANNING]**  
Gait Engine + Body Pose → Foot Trajectories (Cartesian)  

**[ACTUATION]**  
Foot Positions → IK Solver → Joint Angles → Calibration → PWM → Servos

**[FEEDBACK]**  
Ground Sensors → Gait Engine → Adaptive Height Adjustment

> 📖 **For detailed architecture**, see [Technical Reference](docs/technical_reference.md)

---


## Hardware Requirements

### Bill of Materials

| Component | Quantity | Specifications | Notes |
|-----------|----------|----------------|-------|
| STM32F446RE Nucleo | 1 | 180MHz Cortex-M4F | Or equivalent F4 board |
| MG996R Servo | 18 | 13kg-cm @ 6V | Digital servo recommended |
| MPU6050 Module | 1 | GY-521 breakout | Verify I2C address (0x68) |
| Microswitch | 6 | NO type, 3-pin | Foot contact detection |
| NRF24L01+ | 1 | 2.4GHz transceiver | Optional (alternative: iBUS) |
| iBUS Receiver | 1 | Flysky protocol | Optional (alternative: NRF) |
| 2S LiPo Battery | 1 | 7.4V, 2200mAh+ | 30C discharge rate minimum |
| 3D Printed Parts | - | https://github.com/MakeYourPet/hexapod | PLA or PETG material |

**Estimated Cost**: ~$150-200 USD (excluding tools)

**NOTE**: Thanks to the software design, you can use different MCUs, motors, or IMUs by only changing the Hardware Abstraction Layer.

---

## Documentation

### Complete Technical Reference

**Single comprehensive document**: [Technical Reference](docs/technical_reference.md)

**Table of Contents**:
1. **System Architecture** - Layered design, module interactions
2. **Inverse Kinematics** - 3-DOF solver, coordinate transforms, Law of Cosines derivations
3. **Sensor Fusion** - Complementary filter mathematics, frame conventions
4. **Gait Generation** - Phase-driven trajectories, terrain adaptation algorithms
5. **Control System** - FSM modes, PID tuning, real-time scheduler
6. **Calibration Guide** - Step-by-step servo and sensor calibration
7. **Simulation Workflow** - MATLAB/Simulink Model-Based Design validation
8. **Performance Analysis** - Timing measurements, CPU utilization
9. **Hardware Setup** - Wiring diagrams, assembly notes

**For Students**: The documentation is designed to be educational, with:
- Mathematical derivations explained step-by-step
- Design rationale for every major decision
- Worked examples with real numbers
- Troubleshooting guides

**Recommended Reading Path**:
1. System Architecture (understand the big picture)
2. Calibration Guide (get hardware working)
3. Inverse Kinematics (core mathematics)
4. Gait Generation (how walking works)

---

## Project Status

**Current Version**: 2.0 (Complete Rewrite)  
**Status**: ✅ **Stable & Production-Ready**

### What Works

- [x] Layered architecture (HAL/Middleware/Application)
- [x] IK solver (3-DOF)
- [x] Three gait types (Tripod, Wave, Ripple)
- [x] Complementary filter sensor fusion
- [x] Dual-axis PID balance control
- [x] Active terrain adaptation with contact sensors
- [x] Per-servo calibration system
- [x] RC input (NRF24 + iBUS support)
- [x] Real-time scheduler (50Hz/200Hz dual-rate)
- [x] Model-Based Design validation (MATLAB/Simulink)
- [x] Complete technical documentation

### Known Limitations

**Hardware Constraints**:
- Maximum walking speed: ~50mm/s (servo bandwidth limited)
- Terrain height variation: ±40mm (mechanical workspace)
- Maximum slope: ±30° (balance control limit)

### Future Enhancements 

- [ ] Vision-based obstacle detection
- [ ] Machine learning for gait optimization  
- [ ] ROS/ROS2 integration
- [ ] Energy consumption monitoring
- [ ] Multi-robot coordination

---

## Acknowledgments

- MakeYourPet "https://github.com/MakeYourPet/hexapod" for the open-source mechanical design.
  
---

## Contact & Support 

**Questions or Collaboration?**
- **Email**: heisipek@gmail.com
- **LinkedIn**: https://www.linkedin.com/in/ahmet-ipek-a02287217/

---

## Star History

If this project helped you, please consider ⭐ **starring the repository** to help others discover it!

---

**Built with ❤️ for the robotics community**
