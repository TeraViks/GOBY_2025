// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class FaceStationUtil {
  private final FieldPoseUtil m_fieldPoseUtil;

  public FaceStationUtil(FieldPoseUtil fieldPoseUtil) {
    m_fieldPoseUtil = fieldPoseUtil;
  }

  // Get rotation angle, constrained to [-pi..pi]. This is necessary in conjunction with
  // ProfiledPIDController.enableContinuousInput().
  private static Rotation2d rotationAngle(Rotation2d rotation) {
    return new Rotation2d(MathUtil.angleModulus(rotation.getRadians()));
  }

  private Pose2d closestStation(Pose2d robotPose) {
    return robotPose.nearest(m_fieldPoseUtil.getCoralStations());
  }

  public Rotation2d getRotationDeviation(Pose2d robotPose) {
    Rotation2d currentRotation = robotPose.getRotation();
    Rotation2d desiredRotation = closestStation(robotPose).getRotation().plus(Rotation2d.kPi);
    Rotation2d rotationDeviation = rotationAngle(currentRotation.minus(desiredRotation));
    return rotationDeviation;
  }
}