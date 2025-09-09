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

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.questnav.QuestNavSubsystem;

/**
 * Diagnostic command to help debug QuestNav issues. Prints QuestNav status information to the
 * console.
 */
public class QuestNavDiagnostics {

  private QuestNavDiagnostics() {}

  /** Command that prints QuestNav diagnostic information. */
  public static Command printDiagnostics(QuestNavSubsystem questNav, Drive drive) {
    return Commands.runOnce(
        () -> {
          System.out.println("=== QuestNav Diagnostics ===");
          System.out.println("Connected: " + questNav.isActive());
          System.out.println("Tracking: " + questNav.isTracking());

          // Handle battery percentage properly
          var batteryPercent = questNav.getBatteryPercent();
          if (batteryPercent.isPresent()) {
            System.out.println("Battery Percent: " + batteryPercent.getAsInt() + "%");
          } else {
            System.out.println("Battery Percent: N/A");
          }

          // Get yaw data from gyro buffer instead of direct API call
          var gyroInputs = drive.getGyroInputs();
          System.out.println("Gyro Connected: " + gyroInputs.connected);
          System.out.println(
              "Current Yaw (buffered): "
                  + String.format("%.1f", gyroInputs.yawPosition.getDegrees())
                  + "° ("
                  + String.format("%.3f", gyroInputs.yawPosition.getRadians())
                  + " rad)");
          System.out.println(
              "Yaw Velocity: " + String.format("%.4f", gyroInputs.yawVelocityRadPerSec) + " rad/s");
          System.out.println("Odometry Yaw Samples: " + gyroInputs.odometryYawPositions.length);

          try {
            var pose = questNav.getRobotPose();
            System.out.println(
                "Current Pose: X="
                    + String.format("%.2f", pose.getX())
                    + ", Y="
                    + String.format("%.2f", pose.getY())
                    + ", Rotation="
                    + String.format("%.1f", pose.getRotation().getDegrees())
                    + "° ("
                    + String.format("%.3f", pose.getRotation().getRadians())
                    + " rad)");

            var frames = questNav.getAllUnreadPoseFrames();
            System.out.println("Unread Frames: " + frames.length);

            if (frames.length > 0) {
              var latestFrame = frames[frames.length - 1];
              var questPose = latestFrame.questPose();
              System.out.println("Latest Frame Pose: " + questPose);
              System.out.println(
                  "Latest Frame Rotation: "
                      + String.format("%.1f", questPose.getRotation().getDegrees())
                      + "° ("
                      + String.format("%.3f", questPose.getRotation().getRadians())
                      + " rad)");
            }

          } catch (Exception e) {
            System.out.println("Error getting QuestNav data: " + e.getMessage());
            e.printStackTrace();
          }

          System.out.println("==========================");
        },
        questNav,
        drive);
  }
}
