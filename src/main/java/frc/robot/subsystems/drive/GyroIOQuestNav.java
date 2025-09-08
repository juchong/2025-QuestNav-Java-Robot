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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import java.util.Queue;

/** IO implementation for QuestNav. */
public class GyroIOQuestNav implements GyroIO {
  private final QuestNav questNav = new QuestNav();
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;

  // Previous yaw for velocity calculation
  private double previousYawDegrees = 0.0;
  private double previousTimestamp = 0.0;
  private double yawVelocityRadPerSec = 0.0;

  // Last known yaw to prevent flipping to zero
  private double lastKnownYawDegrees = 0.0;

  public GyroIOQuestNav() {
    yawTimestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = SparkOdometryThread.getInstance().registerSignal(this::getCurrentYawDegrees);

    // Initialize previous values
    previousTimestamp = System.currentTimeMillis() / 1000.0;
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    // Update QuestNav
    questNav.commandPeriodic();

    // Get connection status
    inputs.connected = questNav.isConnected();

    // Get current yaw from QuestNav
    double currentYawDegrees = getCurrentYawDegrees();
    double currentTimestamp = System.currentTimeMillis() / 1000.0;

    // Calculate yaw velocity
    if (previousTimestamp > 0.0) {
      double deltaTime = currentTimestamp - previousTimestamp;
      if (deltaTime > 0.0) {
        double deltaYaw = currentYawDegrees - previousYawDegrees;
        // Handle angle wrapping
        while (deltaYaw > 180.0) deltaYaw -= 360.0;
        while (deltaYaw < -180.0) deltaYaw += 360.0;
        yawVelocityRadPerSec = Units.degreesToRadians(deltaYaw / deltaTime);
      }
    }

    // Update previous values
    previousYawDegrees = currentYawDegrees;
    previousTimestamp = currentTimestamp;

    // Set inputs
    inputs.yawPosition = Rotation2d.fromDegrees(currentYawDegrees);
    inputs.yawVelocityRadPerSec = yawVelocityRadPerSec;

    // Update odometry data from queues
    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }

  /**
   * Gets the current yaw angle in degrees from QuestNav.
   *
   * @return Current yaw angle in degrees
   */
  private double getCurrentYawDegrees() {
    PoseFrame[] poseFrames = questNav.getAllUnreadPoseFrames();
    if (poseFrames.length > 0) {
      Pose2d questPose = poseFrames[poseFrames.length - 1].questPose();
      lastKnownYawDegrees = questPose.getRotation().getDegrees();
    }
    return lastKnownYawDegrees; // Return last known value if no new data available
  }
}
