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

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.QuestNavDiagnostics;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOQuestNav;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.questnav.QuestNavSubsystem;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final QuestNavSubsystem questNav;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // Drive mode state
  private boolean questNavMode = true;
  private boolean smoothDriveActive = false;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Initialize QuestNav subsystem
    questNav = new QuestNavSubsystem();

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOQuestNav(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();

    // Initialize logging
    Logger.recordOutput("RobotContainer/QuestNavMode", questNavMode);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Set up drive mode toggle command
    Command toggleDriveMode =
        Commands.runOnce(
            () -> {
              questNavMode = !questNavMode;
              Logger.recordOutput("RobotContainer/QuestNavMode", questNavMode);
              if (questNavMode) {
                System.out.println("QuestNav-enhanced drive mode enabled");
              } else {
                System.out.println("Open-loop drive mode enabled");
              }
              updateDriveCommand();
            });

    // Default command, QuestNav-enhanced drive mode
    drive.setDefaultCommand(
        DriveCommands.questNavJoystickDrive(
            drive,
            questNav,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Toggle between QuestNav-enhanced and open-loop drive when start button (button 9) is pressed
    controller.start().onTrue(toggleDriveMode);

    // Switch to smooth drive mode when X button is pressed (single press toggle)
    // Note: X and Y are swapped on this controller
    controller
        .x()
        .onTrue(
            Commands.runOnce(
                () -> {
                  System.out.println("X button pressed - toggling smooth drive mode");
                  smoothDriveActive = !smoothDriveActive;
                  if (smoothDriveActive) {
                    System.out.println("Smooth drive mode activated");
                  } else {
                    System.out.println("Smooth drive mode deactivated");
                  }
                  updateDriveCommand();
                }));

    // Print QuestNav diagnostics when Y button is pressed
    // Note: Y and X are swapped on this controller
    controller
        .y()
        .onTrue(
            Commands.runOnce(
                    () -> {
                      System.out.println("Y button pressed - printing diagnostics");
                    })
                .andThen(QuestNavDiagnostics.printDiagnostics(questNav, drive)));

    // Reset QuestNav heading when B button is pressed
    // Note: B and A are swapped on this controller
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                () -> {
                  if (questNav.isActive()) {
                    // Reset QuestNav heading to current robot heading
                    Pose2d currentRobotPose = drive.getPose();
                    questNav.setRobotPose(currentRobotPose);
                    System.out.println(
                        "QuestNav heading reset to robot pose: X="
                            + String.format("%.1f", currentRobotPose.getX())
                            + ", Y="
                            + String.format("%.1f", currentRobotPose.getY())
                            + ", Rotation="
                            + String.format("%.1f", currentRobotPose.getRotation().getDegrees())
                            + "°");
                  } else {
                    System.out.println("QuestNav not active - cannot reset heading");
                  }
                }));

    // Reset gyro to 0° when A button is pressed (short press)
    // Note: A and B are swapped on this controller
    controller
        .a()
        .onTrue(
            Commands.runOnce(
                    () -> {
                      drive.setGyroOffset();
                      System.out.println("Gyro zeroed at current heading");
                    },
                    drive)
                .ignoringDisable(true));

    // Note: Long press functionality temporarily disabled due to button mapping conflicts
    // TODO: Re-implement long press with proper button mapping
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  /**
   * Get the current drive mode state.
   *
   * @return true if QuestNav mode is active (default), false if open-loop mode
   */
  public boolean isQuestNavMode() {
    return questNavMode;
  }

  /** Updates the drive command based on current mode settings. */
  private void updateDriveCommand() {
    if (smoothDriveActive) {
      // Use smooth drive mode
      System.out.println("Setting drive command: QuestNav Smooth Drive");
      drive.setDefaultCommand(
          DriveCommands.questNavSmoothDrive(
              drive,
              questNav,
              () -> -controller.getLeftY(),
              () -> -controller.getLeftX(),
              () -> -controller.getRightX()));
    } else if (questNavMode) {
      // Use QuestNav-enhanced drive mode
      System.out.println("Setting drive command: QuestNav-enhanced Drive");
      drive.setDefaultCommand(
          DriveCommands.questNavJoystickDrive(
              drive,
              questNav,
              () -> -controller.getLeftY(),
              () -> -controller.getLeftX(),
              () -> -controller.getRightX()));
    } else {
      // Use open-loop drive mode
      System.out.println("Setting drive command: Open-loop Drive");
      drive.setDefaultCommand(
          DriveCommands.joystickDrive(
              drive,
              () -> -controller.getLeftY(),
              () -> -controller.getLeftX(),
              () -> -controller.getRightX()));
    }
  }
}
