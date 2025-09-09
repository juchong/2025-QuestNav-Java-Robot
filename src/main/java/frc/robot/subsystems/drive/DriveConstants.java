// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class DriveConstants {
  public static final double maxSpeedMetersPerSec = 4.8;
  public static final double odometryFrequency = 100.0; // Hz
  public static final double trackWidth = 0.395;
  public static final double wheelBase = 0.370;
  public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
      };

  // Zeroed rotation values for each module, see setup instructions
  public static final Rotation2d frontLeftZeroRotation = new Rotation2d(1.569);
  public static final Rotation2d frontRightZeroRotation = new Rotation2d(3.089);
  public static final Rotation2d backLeftZeroRotation = new Rotation2d(-0.043);
  public static final Rotation2d backRightZeroRotation = new Rotation2d(-1.569);

  // Device CAN IDs
  public static final int frontLeftDriveCanId = 41;
  public static final int backLeftDriveCanId = 31;
  public static final int frontRightDriveCanId = 11;
  public static final int backRightDriveCanId = 21;

  public static final int frontLeftTurnCanId = 42;
  public static final int backLeftTurnCanId = 32;
  public static final int frontRightTurnCanId = 12;
  public static final int backRightTurnCanId = 22;

  // Not needed for this example, but exists to make it happy
  public static final int pigeonCanId = 99;

  // Drive motor configuration
  public static final int driveMotorCurrentLimit = 20;
  public static final double driveMotorPinionTeeth = 12.0;
  public static final double wheelRadiusMeters = Units.inchesToMeters(1.5);
  // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the
  // bevel pinion
  public static final double driveMotorReduction =
      (45.0 * 22.0)
          / (driveMotorPinionTeeth * 15.0); // MAXSwerve with 12 pinion teeth and 22 spur teeth
  public static final DCMotor driveGearbox = DCMotor.getNEO(1);

  // Drive encoder configuration
  public static final double driveEncoderPositionFactor =
      2 * Math.PI / driveMotorReduction; // Rotor Rotations -> Wheel Radians
  public static final double driveEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM -> Wheel Rad/Sec

  // Drive PID configuration - Written to the motor controllers - Do not change these values!
  public static final double driveKp = 0.0;
  public static final double driveKd = 0.0;
  public static final double driveKs = 0.0;
  public static final double driveKv = 0.1;
  public static final double driveSimP = 0.05;
  public static final double driveSimD = 0.0;
  public static final double driveSimKs = 0.0;
  public static final double driveSimKv = 0.0789;

  // Turn motor configuration
  public static final boolean turnInverted = false;
  public static final int turnMotorCurrentLimit = 20;
  public static final double turnMotorReduction = 9424.0 / 203.0;
  public static final DCMotor turnGearbox = DCMotor.getNeo550(1);

  // Turn encoder configuration
  public static final boolean turnEncoderInverted = true;
  public static final double turnEncoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
  public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

  // Turn PID configuration - Written to the motor controllers - Do not change these values!
  public static final double turnKp = 2.0;
  public static final double turnKd = 0.0;
  public static final double turnSimP = 8.0;
  public static final double turnSimD = 0.0;
  public static final double turnPIDMinInput = 0; // Radians
  public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

  // PathPlanner configuration
  public static final double robotMassKg = 16.16;
  public static final double robotMOI = 0.680; // Estimated
  public static final double wheelCOF = 1.2;
  public static final RobotConfig ppConfig =
      new RobotConfig(
          robotMassKg,
          robotMOI,
          new ModuleConfig(
              wheelRadiusMeters,
              maxSpeedMetersPerSec,
              wheelCOF,
              driveGearbox.withReduction(driveMotorReduction),
              driveMotorCurrentLimit,
              1),
          moduleTranslations);

  // PathPlanner PID configuration
  public static final double pathPlannerTranslationKp = 5.0;
  public static final double pathPlannerTranslationKi = 0.0;
  public static final double pathPlannerTranslationKd = 0.0;
  public static final double pathPlannerRotationKp = 5.0;
  public static final double pathPlannerRotationKi = 0.0;
  public static final double pathPlannerRotationKd = 0.0;

  // QuestNav Closed-Loop Control PID configuration
  public static final double headingKp = 2.0;
  public static final double headingKi = 0.0;
  public static final double headingKd = 0.1;

  public static final double balanceKp = 0.5;
  public static final double balanceKi = 0.0;
  public static final double balanceKd = 0.05;

  public static final double profiledHeadingKp = 3.0;
  public static final double profiledHeadingKi = 0.0;
  public static final double profiledHeadingKd = 0.2;
  public static final double maxHeadingVelocity = 4.0; // rad/s
  public static final double maxHeadingAcceleration = 8.0; // rad/s²

  // Rate limiters for smooth control - VERY CONSERVATIVE SETTINGS
  public static final double linearRateLimit = 2.0; // m/s²
  public static final double angularRateLimit = 1.0; // rad/s²

  // Drive Commands PID configuration
  public static final double angleKp = 5.0;
  public static final double angleKd = 0.4;
  public static final double angleMaxVelocity = 8.0;
  public static final double angleMaxAcceleration = 20.0;

  // Drive Commands control constants
  public static final double deadband = 0.1;
  public static final double ffStartDelay = 2.0; // Secs
  public static final double ffRampRate = 0.1; // Volts/Sec
  public static final double wheelRadiusMaxVelocity = 0.25; // Rad/Sec

  // Inline PID constants for specific commands
  public static final double inlineHeadingKp1 = 1.0 / 60.0;
  public static final double inlineHeadingKi1 = 0.0;
  public static final double inlineHeadingKd1 = 0.001;
  public static final double inlineHeadingTolerance = 0.05; // radians

  // QuestNav filtering constants
  public static final boolean enableQuestNavHeadingFilter =
      true; // Set to false to disable filtering
  public static final double questNavHeadingFilterAlpha =
      0.1; // Much more aggressive filtering (0.0-1.0, lower = more filtering)

  // Additional anti-oscillation constants
  public static final double maxHeadingCorrection =
      0.1; // Maximum correction in rad/s (reduced from 0.3)
  public static final double headingVelocityLimit = 0.5; // Maximum angular velocity in rad/s
  public static final double headingHysteresis =
      0.05; // Hysteresis band in radians to prevent oscillation

  public static final double inlineHeadingKp2 = 2.0;
  public static final double inlineHeadingKi2 = 0.0;
  public static final double inlineHeadingKd2 = 0.15;

  public static final double inlineProfiledHeadingKp = 3.0;
  public static final double inlineProfiledHeadingKi = 0.0;
  public static final double inlineProfiledHeadingKd = 0.2;
  public static final double inlineMaxHeadingVelocity = 4.0;
  public static final double inlineMaxHeadingAcceleration = 8.0;
}
