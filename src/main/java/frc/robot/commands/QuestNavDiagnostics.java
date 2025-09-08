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
import frc.robot.subsystems.questnav.QuestNavSubsystem;

/**
 * Diagnostic command to help debug QuestNav issues. Prints QuestNav status information to the
 * console.
 */
public class QuestNavDiagnostics {

  private QuestNavDiagnostics() {}

  /** Command that prints QuestNav diagnostic information. */
  public static Command printDiagnostics(QuestNavSubsystem questNav) {
    return Commands.runOnce(
        () -> {
          System.out.println("=== QuestNav Diagnostics ===");
          System.out.println("Connected: " + questNav.isActive());
          System.out.println("Tracking: " + questNav.isTracking());

          try {
            double yawDegrees = questNav.getCurrentYawDegrees();
            double yawRadians = questNav.getCurrentYawRadians();
            System.out.println(
                "Current Yaw: " + yawDegrees + " degrees (" + yawRadians + " radians)");

            var pose = questNav.getRobotPose();
            System.out.println(
                "Current Pose: X="
                    + pose.getX()
                    + ", Y="
                    + pose.getY()
                    + ", Rotation="
                    + pose.getRotation().getDegrees()
                    + "° ("
                    + pose.getRotation().getRadians()
                    + " rad)");

            var frames = questNav.getAllUnreadPoseFrames();
            System.out.println("Unread Frames: " + frames.length);

            if (frames.length > 0) {
              var latestFrame = frames[frames.length - 1];
              var questPose = latestFrame.questPose();
              System.out.println("Latest Frame Pose: " + questPose);
              System.out.println(
                  "Latest Frame Rotation: "
                      + questPose.getRotation().getDegrees()
                      + "° ("
                      + questPose.getRotation().getRadians()
                      + " rad)");
            }

          } catch (Exception e) {
            System.out.println("Error getting QuestNav data: " + e.getMessage());
            e.printStackTrace();
          }

          System.out.println("==========================");
        },
        questNav);
  }
}
