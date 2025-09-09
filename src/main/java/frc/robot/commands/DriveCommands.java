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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.questnav.QuestNavSubsystem;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveCommands {
  // Constants are now defined in DriveConstants.java
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

  // Static rate limiters that persist across all drive commands
  private static final SlewRateLimiter staticLinearRateLimiter =
      new SlewRateLimiter(DriveConstants.linearRateLimit);
  private static final SlewRateLimiter staticAngularRateLimiter =
      new SlewRateLimiter(DriveConstants.angularRateLimit);

  private DriveCommands() {}

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband to individual components
    double xDeadbanded = MathUtil.applyDeadband(x, DriveConstants.deadband);
    double yDeadbanded = MathUtil.applyDeadband(y, DriveConstants.deadband);

    // Square individual components for more precise control
    xDeadbanded = Math.copySign(xDeadbanded * xDeadbanded, xDeadbanded);
    yDeadbanded = Math.copySign(yDeadbanded * yDeadbanded, yDeadbanded);

    // Return linear velocity with separate X and Y components
    return new Translation2d(xDeadbanded, yDeadbanded);
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // Apply rotation deadband
          double omega =
              MathUtil.applyDeadband(-omegaSupplier.getAsDouble(), DriveConstants.deadband);

          // Square rotation value for more precise control
          omega = Math.copySign(omega * omega, omega);

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec());
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
        drive);
  }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   */
  public static Command joystickDriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {

    // Create PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            DriveConstants.angleKp,
            0.0,
            DriveConstants.angleKd,
            new TrapezoidProfile.Constraints(
                DriveConstants.angleMaxVelocity, DriveConstants.angleMaxAcceleration));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construct command
    return Commands.run(
            () -> {
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Calculate angular speed
              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
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
            drive)

        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
  }

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Allow modules to orient
        Commands.run(
                () -> {
                  drive.runCharacterization(0.0);
                },
                drive)
            .withTimeout(DriveConstants.ffStartDelay),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
                () -> {
                  double voltage = timer.get() * DriveConstants.ffRampRate;
                  drive.runCharacterization(voltage);
                  velocitySamples.add(drive.getFFCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                drive)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(DriveConstants.wheelRadiusMaxVelocity);
                  drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                    () -> {
                      var rotation = drive.getRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius =
                          (state.gyroDelta * DriveConstants.driveBaseRadius) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }

  // ==================== QuestNav Closed-Loop Drive Commands ====================

  /**
   * Enhanced joystick drive with QuestNav closed-loop heading control. Automatically corrects for
   * heading drift while allowing manual rotation input.
   *
   * <p>Features aggressive anti-oscillation measures: - Very low PID gains for stability - Optional
   * low-pass filtering of QuestNav heading data (configurable in DriveConstants) - Large deadbands
   * to prevent micro-corrections - Velocity limiting to prevent rapid changes - Oscillation
   * detection with automatic safety disable
   */
  public static Command questNavJoystickDrive(
      Drive drive,
      QuestNavSubsystem questNav,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {

    PIDController headingController =
        new PIDController(
            DriveConstants.inlineHeadingKp1,
            DriveConstants.inlineHeadingKi1,
            DriveConstants.inlineHeadingKd1);
    headingController.enableContinuousInput(-Math.PI, Math.PI);
    headingController.setTolerance(DriveConstants.inlineHeadingTolerance); // 6 degree tolerance

    // Low-pass filter for QuestNav heading data
    final double[] filteredHeading = {0.0};
    final boolean[] headingFilterInitialized = {false};

    // Oscillation detection and safety
    final double[] lastHeadingCorrections = {
      0.0, 0.0, 0.0
    }; // Last 3 corrections for oscillation detection
    final int[] correctionIndex = {0};
    final boolean[] questNavDisabled = {false};

    // Use static rate limiters that persist across all drive commands

    return Commands.run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // Apply rotation deadband
          double omega =
              MathUtil.applyDeadband(-omegaSupplier.getAsDouble(), DriveConstants.deadband);

          // If manual rotation input is provided, use it directly
          if (Math.abs(omega) > 0.1) {
            // Manual rotation - square for better control
            omega = Math.copySign(omega * omega, omega);
            omega = staticAngularRateLimiter.calculate(omega);
          } else {
            // No manual rotation - use QuestNav for heading hold (if available and valid)
            if (questNav.isActive() && questNav.isTracking() && !questNavDisabled[0]) {
              try {
                double rawHeading = questNav.getCurrentYawRadians();

                // Apply low-pass filter to reduce noise (if enabled)
                double currentHeading;
                if (DriveConstants.enableQuestNavHeadingFilter) {
                  if (!headingFilterInitialized[0]) {
                    filteredHeading[0] = rawHeading;
                    headingFilterInitialized[0] = true;
                  } else {
                    // Exponential moving average filter
                    filteredHeading[0] =
                        DriveConstants.questNavHeadingFilterAlpha * rawHeading
                            + (1.0 - DriveConstants.questNavHeadingFilterAlpha)
                                * filteredHeading[0];
                  }
                  currentHeading = filteredHeading[0];
                } else {
                  // Filter disabled - use raw heading data
                  currentHeading = rawHeading;
                }

                // Only apply correction if we have valid data and are not already at setpoint
                if (Math.abs(currentHeading) > 0.001
                    || questNav.getRobotPose().getTranslation().getNorm() > 0.001) {

                  // Much larger deadband to prevent small corrections that cause oscillations
                  if (Math.abs(currentHeading) > DriveConstants.inlineHeadingTolerance) {
                    double headingCorrection = headingController.calculate(currentHeading, 0.0);

                    // Apply much larger deadband to correction to prevent micro-corrections
                    if (Math.abs(headingCorrection)
                        > 0.05) { // ~3 degree deadband (increased from 1 degree)
                      // Much more aggressive correction limiting
                      headingCorrection =
                          MathUtil.clamp(
                              headingCorrection,
                              -DriveConstants.maxHeadingCorrection,
                              DriveConstants.maxHeadingCorrection);

                      // Store correction for oscillation detection
                      lastHeadingCorrections[correctionIndex[0]] = headingCorrection;
                      correctionIndex[0] = (correctionIndex[0] + 1) % 3;

                      // Check for oscillation pattern (alternating signs with large magnitude)
                      if (correctionIndex[0] == 0) { // We have 3 samples
                        double avgMagnitude =
                            (Math.abs(lastHeadingCorrections[0])
                                    + Math.abs(lastHeadingCorrections[1])
                                    + Math.abs(lastHeadingCorrections[2]))
                                / 3.0;
                        boolean alternating =
                            (lastHeadingCorrections[0] * lastHeadingCorrections[1] < 0)
                                && (lastHeadingCorrections[1] * lastHeadingCorrections[2] < 0);

                        if (alternating && avgMagnitude > 0.05) {
                          // Oscillation detected - disable QuestNav heading control
                          questNavDisabled[0] = true;
                          System.out.println(
                              "WARNING: QuestNav heading oscillation detected! Disabling heading control.");
                          omega = 0.0;
                        } else {
                          // Apply velocity limiting to prevent rapid changes
                          omega =
                              MathUtil.clamp(
                                  staticAngularRateLimiter.calculate(headingCorrection),
                                  -DriveConstants.headingVelocityLimit,
                                  DriveConstants.headingVelocityLimit);
                        }
                      } else {
                        // Apply velocity limiting to prevent rapid changes
                        omega =
                            MathUtil.clamp(
                                staticAngularRateLimiter.calculate(headingCorrection),
                                -DriveConstants.headingVelocityLimit,
                                DriveConstants.headingVelocityLimit);
                      }
                    } else {
                      omega = 0.0;
                    }
                  } else {
                    // Close enough to setpoint, no correction needed
                    omega = 0.0;
                  }
                } else {
                  // QuestNav data appears invalid, don't apply correction
                  omega = 0.0;
                }
              } catch (Exception e) {
                // QuestNav not available, use manual control
                omega = 0.0;
              }
            } else {
              // QuestNav not active or not tracking, no automatic correction
              omega = 0.0;
            }
          }

          // Apply rate limiting to linear movement (disabled for now)
          double xSpeed = linearVelocity.getX();
          double ySpeed = linearVelocity.getY();

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  xSpeed * drive.getMaxLinearSpeedMetersPerSec(),
                  ySpeed * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec());

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
   * QuestNav-enhanced drive with automatic heading correction. Maintains straight-line movement
   * when no rotation input is provided.
   */
  public static Command questNavStraightDrive(
      Drive drive,
      QuestNavSubsystem questNav,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {

    PIDController headingController =
        new PIDController(
            DriveConstants.inlineHeadingKp2,
            DriveConstants.inlineHeadingKi2,
            DriveConstants.inlineHeadingKd2);
    headingController.enableContinuousInput(-Math.PI, Math.PI);

    // Use static rate limiters that persist across all drive commands

    return Commands.run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // Apply rotation deadband
          double omega =
              MathUtil.applyDeadband(-omegaSupplier.getAsDouble(), DriveConstants.deadband);

          // Always apply QuestNav heading correction, but allow manual override
          if (questNav.isActive()) {
            try {
              double currentHeading = questNav.getCurrentYawRadians();
              double headingCorrection = headingController.calculate(currentHeading, 0.0);

              // Combine manual rotation with heading correction
              omega = omega + (headingCorrection * 0.3); // Scale down auto-correction
            } catch (Exception e) {
              // QuestNav not available, use manual control only
            }
          }
          omega = MathUtil.clamp(omega, -1.0, 1.0); // Clamp to prevent saturation
          omega = staticAngularRateLimiter.calculate(omega);

          // Apply rate limiting to linear movement
          double xSpeed = staticLinearRateLimiter.calculate(linearVelocity.getX());
          double ySpeed = staticLinearRateLimiter.calculate(linearVelocity.getY());

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  xSpeed * drive.getMaxLinearSpeedMetersPerSec(),
                  ySpeed * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec());
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
   * QuestNav-enhanced drive with smooth rotation control. Uses profiled PID for smooth heading
   * changes when rotation input is provided.
   */
  public static Command questNavSmoothDrive(
      Drive drive,
      QuestNavSubsystem questNav,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {

    ProfiledPIDController headingController =
        new ProfiledPIDController(
            DriveConstants.inlineProfiledHeadingKp,
            DriveConstants.inlineProfiledHeadingKi,
            DriveConstants.inlineProfiledHeadingKd,
            new TrapezoidProfile.Constraints(
                DriveConstants.inlineMaxHeadingVelocity,
                DriveConstants.inlineMaxHeadingAcceleration));
    headingController.enableContinuousInput(-Math.PI, Math.PI);

    // Use static rate limiters that persist across all drive commands

    return Commands.run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // Apply rotation deadband
          double omega =
              MathUtil.applyDeadband(-omegaSupplier.getAsDouble(), DriveConstants.deadband);

          // If manual rotation input is provided, use profiled PID for smooth control
          if (questNav.isActive()) {
            try {
              if (Math.abs(omega) > 0.1) {
                // Calculate target heading based on current heading + rotation input
                double currentHeading = questNav.getCurrentYawRadians();
                double targetHeading = currentHeading + (omega * 0.5); // Scale rotation input
                omega = headingController.calculate(currentHeading, targetHeading);
                omega = staticAngularRateLimiter.calculate(omega);
              } else {
                // No manual rotation - hold current heading
                double currentHeading = questNav.getCurrentYawRadians();
                omega = headingController.calculate(currentHeading, currentHeading);
                omega = staticAngularRateLimiter.calculate(omega);
              }
            } catch (Exception e) {
              // QuestNav not available, use manual control
              omega = Math.copySign(omega * omega, omega);
              omega = staticAngularRateLimiter.calculate(omega);
            }
          } else {
            // QuestNav not active, use manual control
            omega = Math.copySign(omega * omega, omega);
            omega = staticAngularRateLimiter.calculate(omega);
          }

          // Apply rate limiting to linear movement
          double xSpeed = staticLinearRateLimiter.calculate(linearVelocity.getX());
          double ySpeed = staticLinearRateLimiter.calculate(linearVelocity.getY());

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  xSpeed * drive.getMaxLinearSpeedMetersPerSec(),
                  ySpeed * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec());
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

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = new Rotation2d();
    double gyroDelta = 0.0;
  }
}
