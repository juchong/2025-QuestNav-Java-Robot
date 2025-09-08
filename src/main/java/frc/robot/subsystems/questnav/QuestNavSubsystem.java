// Copyright QuestNav 2025
// https://github.com/QuestNav/QuestNav
// https://questnav.gg/
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

package frc.robot.subsystems.questnav;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import org.littletonrobotics.junction.Logger;

/**
 * QuestNav subsystem for vision-based pose estimation.
 *
 * <p>This subsystem integrates QuestNav vision tracking with the robot's pose estimation. It
 * provides methods to get robot pose from QuestNav and set robot pose in QuestNav.
 */
public class QuestNavSubsystem extends SubsystemBase {
  private final QuestNav questNav;

  // Transform from robot center to QuestNav camera position
  // TODO: Measure and set the actual transform values
  private static final Transform2d ROBOT_TO_QUEST = new Transform2d(0.0, 0.0, null);

  public QuestNavSubsystem() {
    questNav = new QuestNav();
  }

  @Override
  public void periodic() {
    questNav.commandPeriodic();

    // Log QuestNav data
    Logger.recordOutput("QuestNav/Connected", questNav.isConnected());
    Logger.recordOutput("QuestNav/Tracking", questNav.isTracking());
    Logger.recordOutput("QuestNav/Latency", questNav.getLatency());

    // Log battery percentage if available
    questNav
        .getBatteryPercent()
        .ifPresent(battery -> Logger.recordOutput("QuestNav/BatteryPercent", battery));

    // Log frame count if available
    questNav
        .getFrameCount()
        .ifPresent(frameCount -> Logger.recordOutput("QuestNav/FrameCount", frameCount));
  }

  /**
   * Gets the latest robot pose from QuestNav.
   *
   * @return The robot's current pose, or a default pose if no data is available
   */
  public Pose2d getRobotPose() {
    PoseFrame[] poseFrames = questNav.getAllUnreadPoseFrames();
    if (poseFrames.length > 0) {
      Pose2d questPose = poseFrames[poseFrames.length - 1].questPose();
      return questPose.transformBy(ROBOT_TO_QUEST.inverse());
    }
    return new Pose2d(); // Return default pose if no data is available
  }

  /**
   * Sets the robot's pose in QuestNav.
   *
   * @param robotPose The robot's pose to set in QuestNav
   */
  public void setRobotPose(Pose2d robotPose) {
    Pose2d questPose = robotPose.transformBy(ROBOT_TO_QUEST);
    questNav.setPose(questPose);
  }

  /**
   * Checks if QuestNav is connected and active.
   *
   * @return true if QuestNav is connected, false otherwise
   */
  public boolean isActive() {
    return questNav.isConnected();
  }

  /**
   * Checks if QuestNav is currently tracking.
   *
   * @return true if QuestNav is tracking, false otherwise
   */
  public boolean isTracking() {
    return questNav.isTracking();
  }

  /**
   * Gets all unread pose frames from QuestNav.
   *
   * @return Array of unread pose frames
   */
  public PoseFrame[] getAllUnreadPoseFrames() {
    return questNav.getAllUnreadPoseFrames();
  }

  /**
   * Gets the current latency of QuestNav in seconds.
   *
   * @return Latency in seconds
   */
  public double getLatency() {
    return questNav.getLatency();
  }

  /**
   * Gets the battery percentage of the QuestNav device.
   *
   * @return Optional containing battery percentage if available
   */
  public java.util.OptionalInt getBatteryPercent() {
    return questNav.getBatteryPercent();
  }

  /**
   * Gets the current frame count from QuestNav.
   *
   * @return Optional containing frame count if available
   */
  public java.util.OptionalInt getFrameCount() {
    return questNav.getFrameCount();
  }

  /**
   * Gets the tracking lost counter from QuestNav.
   *
   * @return Optional containing tracking lost counter if available
   */
  public java.util.OptionalInt getTrackingLostCounter() {
    return questNav.getTrackingLostCounter();
  }

  /**
   * Gets the app timestamp from QuestNav.
   *
   * @return Optional containing app timestamp if available
   */
  public java.util.OptionalDouble getAppTimestamp() {
    return questNav.getAppTimestamp();
  }
}
