# 2025 QuestNav Java Robot

A FIRST Robotics Competition (FRC) robot code project featuring QuestNav vision-based pose estimation integrated with a swerve drive system. This robot demonstrates advanced autonomous navigation capabilities using QuestNav's computer vision technology.

## 🚀 Features

- **QuestNav Vision System**: Advanced pose estimation using QuestNav's computer vision technology
- **Swerve Drive**: Four-wheel independent steering and driving for omnidirectional movement
- **Path Planning**: Integration with PathPlanner for autonomous path following
- **AdvantageKit Logging**: Comprehensive data logging and analysis capabilities
- **Simulation Support**: Full physics simulation support for testing and development
- **Closed-Loop Control**: PID-based heading control and balance control systems

## 🏗️ Robot Specifications

### Drive System
- **Type**: Swerve Drive (4 modules)
- **Max Speed**: 4.8 m/s
- **Track Width**: 0.395m
- **Wheel Base**: 0.370m
- **Wheel Radius**: 1.5 inches
- **Motors**: NEO brushless motors for drive, NEO 550 for steering
- **Encoders**: Integrated encoders with gear reduction

### QuestNav Integration
- **Vision System**: QuestNav computer vision for pose estimation
- **Tracking**: Real-time robot pose tracking with sub-inch accuracy
- **Latency**: Low-latency pose updates for responsive control
- **Battery Monitoring**: QuestNav device battery level tracking

## 🛠️ Technology Stack

### Core Libraries
- **WPILib 2025.3.2**: FRC robotics framework
- **AdvantageKit 4.1.2**: Advanced logging and analysis
- **QuestNavLib 2025-1.1.1-beta**: Vision-based pose estimation
- **PathPlanner**: Autonomous path planning and following
- **CTRE Phoenix 6**: Motor controller libraries
- **REVLib**: REV Robotics motor controller support

### Development Tools
- **Java 17**: Programming language
- **Gradle**: Build system
- **Spotless**: Code formatting
- **JUnit 5**: Testing framework

## 📁 Project Structure

```
src/main/java/frc/robot/
├── Robot.java                 # Main robot class
├── RobotContainer.java        # Robot configuration and controls
├── Constants.java             # Runtime mode configuration
├── commands/
│   ├── DriveCommands.java     # Drive control commands
│   ├── QuestNavDiagnostics.java
│   └── QuestNavClosedLoopControl.java
└── subsystems/
    ├── drive/                 # Swerve drive subsystem
    │   ├── Drive.java
    │   ├── DriveConstants.java
    │   ├── Module.java
    │   └── GyroIO*.java       # Gyroscope implementations
    └── questnav/              # QuestNav vision subsystem
        └── QuestNavSubsystem.java
```

## 🎮 Controls

### Driver Controls (Xbox Controller)
- **Left Stick**: Forward/backward and strafe movement
- **Right Stick**: Rotation control
- **A Button**: Toggle QuestNav mode
- **B Button**: Toggle smooth drive mode
- **X Button**: Reset robot pose
- **Y Button**: QuestNav diagnostics
- **Start Button**: Reset gyroscope
- **Back Button**: Reset pose estimation

### Autonomous
- **PathPlanner Integration**: Pre-programmed autonomous paths
- **QuestNav Pose Estimation**: Vision-based autonomous navigation
- **Closed-Loop Control**: PID-based heading and balance control

## 🔧 Setup Instructions

### Prerequisites
1. **WPILib 2025.3.2** or later
2. **Java 17** or later
3. **Gradle** (included with WPILib)
4. **QuestNav Device** with QuestNavLib integration

### Installation
1. Clone the repository:
   ```bash
   git clone https://github.com/your-team/2025-QuestNav-Java-Robot.git
   cd 2025-QuestNav-Java-Robot
   ```

2. Open the project in VS Code with the WPILib extension

3. Configure your team number in `.wpilib/wpilib_preferences.json`

4. Set up QuestNav device according to QuestNav documentation

### Hardware Configuration
- **CAN IDs**: Configured in `DriveConstants.java`
  - Drive Motors: 11, 21, 31, 41
  - Turn Motors: 12, 22, 32, 42
  - Pigeon IMU: 99

## 🚀 Building and Deploying

### Build
```bash
./gradlew build
```

### Deploy to Robot
```bash
./gradlew deploy
```

### Run Simulation
```bash
./gradlew simulate
```

### Run Tests
```bash
./gradlew test
```

## 📊 Data Logging

This project uses AdvantageKit for comprehensive data logging:

- **Real Robot**: Logs to USB drive at `/U/logs`
- **Simulation**: Logs to NetworkTables
- **Replay**: Supports log file replay for analysis

### Logged Data
- QuestNav pose estimation
- Drive system telemetry
- Motor controller data
- Gyroscope readings
- Control system outputs

## 🧪 Testing and Simulation

### Simulation Mode
The robot supports full physics simulation for testing without hardware:
- Swerve drive physics simulation
- QuestNav pose estimation simulation
- PathPlanner integration testing

### QuestNav Diagnostics
Use the QuestNav diagnostics command to:
- Test QuestNav connectivity
- Monitor tracking status
- Verify pose estimation accuracy
- Check device battery levels

## 🔧 Configuration

### Drive Constants
Key parameters in `DriveConstants.java`:
- Motor current limits
- PID controller gains
- Rate limiting values
- QuestNav-specific tuning parameters

### QuestNav Settings
- Camera transform calibration
- Tracking sensitivity
- Pose estimation filtering
- Anti-oscillation parameters

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Run tests and formatting
5. Submit a pull request

### Code Style
This project uses Spotless for code formatting:
- Google Java Format
- Automatic import organization
- Trailing whitespace removal

## 📄 License

This project is licensed under the GNU General Public License v3.0 - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- **Mechanical Advantage** for the AdvantageKit framework
- **QuestNav** for the vision-based pose estimation system
- **WPILib** for the FRC robotics framework
- **PathPlanner** for autonomous path planning capabilities

## 📞 Support

For questions and support:
- Create an issue in this repository
- Contact the development team
- Check the [QuestNav documentation](https://questnav.gg/)

---

**Team**: [Your Team Name]
**Year**: 2025
**Competition**: FIRST Robotics Competition
