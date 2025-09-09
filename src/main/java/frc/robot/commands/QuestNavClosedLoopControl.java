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
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.questnav.QuestNavSubsystem;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Closed-loop control commands using QuestNav for feedback. Provides various control modes
 * including heading hold, heading change, and balance control.
 */
public class QuestNavClosedLoopControl {

  // PID Constants are now defined in DriveConstants.java

  private QuestNavClosedLoopControl() {}

  /**
   * Command to hold the current heading while allowing linear movement. Uses QuestNav yaw data for
   * feedback to maintain heading.
   */
  public static Command holdHeading(
      Drive drive, QuestNavSubsystem questNav, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {

    PIDController headingController =
        new PIDController(
            DriveConstants.headingKp, DriveConstants.headingKi, DriveConstants.headingKd);
    headingController.enableContinuousInput(-Math.PI, Math.PI);

    SlewRateLimiter linearRateLimiter = new SlewRateLimiter(DriveConstants.linearRateLimit);
    SlewRateLimiter angularRateLimiter = new SlewRateLimiter(DriveConstants.angularRateLimit);

    return Commands.run(
        () -> {
          // Get current heading from QuestNav
          double currentHeading = questNav.getRobotPose().getRotation().getRadians();

          // Calculate heading correction
          double headingCorrection = headingController.calculate(currentHeading, 0.0);
          headingCorrection = angularRateLimiter.calculate(headingCorrection);

          // Get linear velocity from joysticks
          double x = MathUtil.applyDeadband(xSupplier.getAsDouble(), DriveConstants.deadband);
          double y = MathUtil.applyDeadband(ySupplier.getAsDouble(), DriveConstants.deadband);

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
            DriveConstants.profiledHeadingKp,
            DriveConstants.profiledHeadingKi,
            DriveConstants.profiledHeadingKd,
            new TrapezoidProfile.Constraints(
                DriveConstants.maxHeadingVelocity, DriveConstants.maxHeadingAcceleration));
    headingController.enableContinuousInput(-Math.PI, Math.PI);

    SlewRateLimiter linearRateLimiter = new SlewRateLimiter(DriveConstants.linearRateLimit);

    return Commands.run(
        () -> {
          // Get current heading from QuestNav
          double currentHeading = questNav.getRobotPose().getRotation().getRadians();
          double targetHeading = targetHeadingSupplier.get().getRadians();

          // Calculate heading correction
          double headingCorrection = headingController.calculate(currentHeading, targetHeading);

          // Get linear velocity from joysticks
          double x = MathUtil.applyDeadband(xSupplier.getAsDouble(), DriveConstants.deadband);
          double y = MathUtil.applyDeadband(ySupplier.getAsDouble(), DriveConstants.deadband);

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

    PIDController pitchController =
        new PIDController(
            DriveConstants.balanceKp, DriveConstants.balanceKi, DriveConstants.balanceKd);
    PIDController rollController =
        new PIDController(
            DriveConstants.balanceKp, DriveConstants.balanceKi, DriveConstants.balanceKd);

    SlewRateLimiter linearRateLimiter = new SlewRateLimiter(DriveConstants.linearRateLimit);

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
          double x = MathUtil.applyDeadband(xSupplier.getAsDouble(), DriveConstants.deadband);
          double y = MathUtil.applyDeadband(ySupplier.getAsDouble(), DriveConstants.deadband);

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

    PIDController headingController =
        new PIDController(
            DriveConstants.headingKp, DriveConstants.headingKi, DriveConstants.headingKd);
    headingController.enableContinuousInput(-Math.PI, Math.PI);

    SlewRateLimiter speedRateLimiter = new SlewRateLimiter(DriveConstants.linearRateLimit);

    return Commands.run(
        () -> {
          // Get current heading from QuestNav
          double currentHeading = questNav.getRobotPose().getRotation().getRadians();
          double targetHeading = targetHeadingSupplier.get().getRadians();

          // Calculate heading correction
          double headingCorrection = headingController.calculate(currentHeading, targetHeading);

          // Get speed from supplier
          double speed =
              MathUtil.applyDeadband(speedSupplier.getAsDouble(), DriveConstants.deadband);
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
            DriveConstants.profiledHeadingKp,
            DriveConstants.profiledHeadingKi,
            DriveConstants.profiledHeadingKd,
            new TrapezoidProfile.Constraints(
                DriveConstants.maxHeadingVelocity, DriveConstants.maxHeadingAcceleration));
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
