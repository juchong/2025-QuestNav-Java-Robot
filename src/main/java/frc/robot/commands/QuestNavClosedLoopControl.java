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

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.questnav.QuestNavSubsystem;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Closed-loop control commands using QuestNav for feedback. Provides various control modes
 * including heading hold, heading change, and balance control.
 */
public class QuestNavClosedLoopControl {

  // PID Constants for heading control
  private static final double HEADING_KP = 2.0;
  private static final double HEADING_KI = 0.0;
  private static final double HEADING_KD = 0.1;

  // PID Constants for balance control
  private static final double BALANCE_KP = 0.5;
  private static final double BALANCE_KI = 0.0;
  private static final double BALANCE_KD = 0.05;

  // Profiled PID Constants for smooth heading changes
  private static final double PROFILED_HEADING_KP = 3.0;
  private static final double PROFILED_HEADING_KI = 0.0;
  private static final double PROFILED_HEADING_KD = 0.2;
  private static final double MAX_HEADING_VELOCITY = 4.0; // rad/s
  private static final double MAX_HEADING_ACCELERATION = 8.0; // rad/s²

  // Rate limiters for smooth control
  private static final double LINEAR_RATE_LIMIT = 2.0; // m/s²
  private static final double ANGULAR_RATE_LIMIT = 3.0; // rad/s²

  private QuestNavClosedLoopControl() {}

  /**
   * Command to hold the current heading while allowing linear movement. Uses QuestNav yaw data for
   * feedback to maintain heading.
   */
  public static Command holdHeading(
      Drive drive, QuestNavSubsystem questNav, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {

    PIDController headingController = new PIDController(HEADING_KP, HEADING_KI, HEADING_KD);
    headingController.enableContinuousInput(-Math.PI, Math.PI);

    SlewRateLimiter linearRateLimiter = new SlewRateLimiter(LINEAR_RATE_LIMIT);
    SlewRateLimiter angularRateLimiter = new SlewRateLimiter(ANGULAR_RATE_LIMIT);

    return Commands.run(
        () -> {
          // Get current heading from QuestNav
          double currentHeading = questNav.getRobotPose().getRotation().getRadians();

          // Calculate heading correction
          double headingCorrection = headingController.calculate(currentHeading, 0.0);
          headingCorrection = angularRateLimiter.calculate(headingCorrection);

          // Get linear velocity from joysticks
          double x = MathUtil.applyDeadband(xSupplier.getAsDouble(), 0.1);
          double y = MathUtil.applyDeadband(ySupplier.getAsDouble(), 0.1);

          // Apply rate limiting to linear movement
          x = linearRateLimiter.calculate(x);
          y = linearRateLimiter.calculate(y);

          // Convert to field relative speeds
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  x * drive.getMaxLinearSpeedMetersPerSec(),
                  y * drive.getMaxLinearSpeedMetersPerSec(),
                  headingCorrection);

          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;

          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive,
        questNav);
  }

  /**
   * Command to drive to a specific heading using QuestNav feedback. Uses profiled PID for smooth
   * heading changes.
   */
  public static Command driveToHeading(
      Drive drive,
      QuestNavSubsystem questNav,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> targetHeadingSupplier) {

    ProfiledPIDController headingController =
        new ProfiledPIDController(
            PROFILED_HEADING_KP,
            PROFILED_HEADING_KI,
            PROFILED_HEADING_KD,
            new TrapezoidProfile.Constraints(MAX_HEADING_VELOCITY, MAX_HEADING_ACCELERATION));
    headingController.enableContinuousInput(-Math.PI, Math.PI);

    SlewRateLimiter linearRateLimiter = new SlewRateLimiter(LINEAR_RATE_LIMIT);

    return Commands.run(
        () -> {
          // Get current heading from QuestNav
          double currentHeading = questNav.getRobotPose().getRotation().getRadians();
          double targetHeading = targetHeadingSupplier.get().getRadians();

          // Calculate heading correction
          double headingCorrection = headingController.calculate(currentHeading, targetHeading);

          // Get linear velocity from joysticks
          double x = MathUtil.applyDeadband(xSupplier.getAsDouble(), 0.1);
          double y = MathUtil.applyDeadband(ySupplier.getAsDouble(), 0.1);

          // Apply rate limiting to linear movement
          x = linearRateLimiter.calculate(x);
          y = linearRateLimiter.calculate(y);

          // Convert to field relative speeds
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  x * drive.getMaxLinearSpeedMetersPerSec(),
                  y * drive.getMaxLinearSpeedMetersPerSec(),
                  headingCorrection);

          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;

          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive,
        questNav);
  }

  /**
   * Command to maintain balance using QuestNav pitch and roll data. Useful for climbing or
   * maintaining level orientation.
   */
  public static Command maintainBalance(
      Drive drive, QuestNavSubsystem questNav, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {

    PIDController pitchController = new PIDController(BALANCE_KP, BALANCE_KI, BALANCE_KD);
    PIDController rollController = new PIDController(BALANCE_KP, BALANCE_KI, BALANCE_KD);

    SlewRateLimiter linearRateLimiter = new SlewRateLimiter(LINEAR_RATE_LIMIT);

    return Commands.run(
        () -> {
          // Extract pitch and roll from QuestNav (assuming they're available in the pose)
          // Note: This is a simplified approach - you may need to access raw IMU data
          double pitch = 0.0; // TODO: Extract actual pitch from QuestNav
          double roll = 0.0; // TODO: Extract actual roll from QuestNav

          // Calculate balance corrections
          double pitchCorrection = pitchController.calculate(pitch, 0.0);
          double rollCorrection = rollController.calculate(roll, 0.0);

          // Get linear velocity from joysticks
          double x = MathUtil.applyDeadband(xSupplier.getAsDouble(), 0.1);
          double y = MathUtil.applyDeadband(ySupplier.getAsDouble(), 0.1);

          // Apply rate limiting to linear movement
          x = linearRateLimiter.calculate(x);
          y = linearRateLimiter.calculate(y);

          // Combine balance corrections with linear movement
          // This is a simplified approach - you may need more sophisticated control
          double omega = (pitchCorrection + rollCorrection) * 0.5;

          // Convert to field relative speeds
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  x * drive.getMaxLinearSpeedMetersPerSec(),
                  y * drive.getMaxLinearSpeedMetersPerSec(),
                  omega);

          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;

          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive,
        questNav);
  }

  /**
   * Command to drive straight using QuestNav heading feedback. Maintains a straight line by
   * correcting for heading drift.
   */
  public static Command driveStraight(
      Drive drive,
      QuestNavSubsystem questNav,
      DoubleSupplier speedSupplier,
      Supplier<Rotation2d> targetHeadingSupplier) {

    PIDController headingController = new PIDController(HEADING_KP, HEADING_KI, HEADING_KD);
    headingController.enableContinuousInput(-Math.PI, Math.PI);

    SlewRateLimiter speedRateLimiter = new SlewRateLimiter(LINEAR_RATE_LIMIT);

    return Commands.run(
        () -> {
          // Get current heading from QuestNav
          double currentHeading = questNav.getRobotPose().getRotation().getRadians();
          double targetHeading = targetHeadingSupplier.get().getRadians();

          // Calculate heading correction
          double headingCorrection = headingController.calculate(currentHeading, targetHeading);

          // Get speed from supplier
          double speed = MathUtil.applyDeadband(speedSupplier.getAsDouble(), 0.1);
          speed = speedRateLimiter.calculate(speed);

          // Convert to field relative speeds
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  speed * drive.getMaxLinearSpeedMetersPerSec(),
                  0.0, // No lateral movement
                  headingCorrection);

          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;

          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive,
        questNav);
  }

  /**
   * Command to turn to a specific heading using QuestNav feedback. Uses profiled PID for smooth
   * rotation.
   */
  public static Command turnToHeading(
      Drive drive, QuestNavSubsystem questNav, Supplier<Rotation2d> targetHeadingSupplier) {

    ProfiledPIDController headingController =
        new ProfiledPIDController(
            PROFILED_HEADING_KP,
            PROFILED_HEADING_KI,
            PROFILED_HEADING_KD,
            new TrapezoidProfile.Constraints(MAX_HEADING_VELOCITY, MAX_HEADING_ACCELERATION));
    headingController.enableContinuousInput(-Math.PI, Math.PI);

    return Commands.run(
            () -> {
              // Get current heading from QuestNav
              double currentHeading = questNav.getRobotPose().getRotation().getRadians();
              double targetHeading = targetHeadingSupplier.get().getRadians();

              // Calculate heading correction
              double headingCorrection = headingController.calculate(currentHeading, targetHeading);

              // Only rotate, no linear movement
              ChassisSpeeds speeds = new ChassisSpeeds(0.0, 0.0, headingCorrection);

              drive.runVelocity(speeds);
            },
            drive,
            questNav)
        .until(() -> Math.abs(headingController.getPositionError()) < 0.05); // Within 3 degrees
  }
}
