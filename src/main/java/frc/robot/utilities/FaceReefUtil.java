// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class FaceReefUtil {
  private final Translation2d m_reef;

  public FaceReefUtil(FieldPoseUtil fieldPoseUtil) {
    m_reef = fieldPoseUtil.getReefCenter();
  }

  // Get rotation angle, constrained to [-pi..pi]. This is necessary in conjunction with
  // ProfiledPIDController.enableContinuousInput().
  private static Rotation2d rotationAngle(Rotation2d rotation) {
    return new Rotation2d(MathUtil.angleModulus(rotation.getRadians()));
  }

  private Translation2d getRobotToReef(Pose2d robotPose) {
    Translation2d robotToReef = m_reef.minus(robotPose.getTranslation());
    return robotToReef;
  }

  // Calculates the desired angle from the robot's current angle
  private Rotation2d getDesiredRotation(Pose2d robotPose) {
    Rotation2d reefToRobotAngle = getRobotToReef(robotPose).getAngle();
    Rotation2d desiredAngle = new Rotation2d(
      Math.floor(
        (reefToRobotAngle.getRadians() + (Math.PI/6.0))
        / (Math.PI/3.0)) * (Math.PI/3.0));
    return desiredAngle;
  }

  public Rotation2d getRotationDeviation(Pose2d robotPose) {
    Rotation2d currentRotation = robotPose.getRotation();
    Rotation2d desiredRotation = getDesiredRotation(robotPose);
    Rotation2d rotationDeviation = rotationAngle(currentRotation.minus(desiredRotation));
    return rotationDeviation;
  }
}