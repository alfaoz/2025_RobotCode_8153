# T4P FRC 2025 Robot Code

## 📝 About
This repository contains the robot code developed for the 2025 FRC season by FRC Team 8153 Tech4Peace. We will be competing at the Marmara Regional (Week 2) from March 7 to March 9, 2025.

## 🛠️ Requirements
- WPILib 2025.2.1 or higher
- Java 17 or higher
- Gradle
- FRC Driver Station
- FRC Game Tools

## 🚀 Installation
1. Clone the repository:
```bash
git clone https://github.com/brownii3/8153_2025_code.git
```

2. Navigate to the project directory:
```bash
cd 8153_2025_code
```

3. Install dependencies:
```bash
./gradlew build
```

## 📦 Build and Deploy
- To build the code:
```bash
./gradlew build
```

- To deploy to the robot:
```bash
./gradlew deploy
```

## 🤖 Robot Features
- Swerve Drive System
- Vision Processing with Limelight
- Autonomous Routines


## 📂 Project Structure
```
src/
├── main/
│   ├── java/
│   │   └── frc/
│   │       └── robot/
│   │           ├── commands/     # Robot commands
│   │           ├── subsystems/   # Subsystems
│   │           ├── Constants.java
│   │           ├── Robot.java
│   │           └── RobotContainer.java
│   └── deploy/                   # Files to be deployed
```

## 🔧 Configuration
- Edit `.wpilib/wpilib_preferences.json` to change the robot number (8153)
- Constants for subsystems can be found in `Constants.java`

## 🤝 Contributing
1. Fork this repository
2. Create a feature branch (`git checkout -b feature/NewFeature`)
3. Commit your changes (`git commit -m 'Add new feature'`)
4. Push to the branch (`git push origin feature/NewFeature`)
5. Create a Pull Request

## ⚠️ Known Issues
- Please check the Issues tab on GitHub for current issues
- Report new issues through GitHub Issues

## 📝 License
This project is licensed under the WPILib License. See `WPILib-License.md` for details.

## 👥 Team
Tech4Peace - FRC Team 8153
- Location: Turkey
- Rookie Year: 2020

## 📞 Contact
- Instagram: [@acihsrobotics](https://www.instagram.com/acihsrobotics/)
