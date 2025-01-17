// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;

public class FaceReefUtil {
    private final Translation2d m_reef;

    public FaceReefUtil() {
        m_reef = getReef(getAlliance());
    }

    private static Alliance getAlliance() {
        Optional<Alliance> allianceOpt = DriverStation.getAlliance();
        Alliance alliance = allianceOpt.isPresent() ? allianceOpt.get() : Alliance.Blue;
        return alliance;
    }

    private static Translation2d getReef(Alliance alliance) {
        Translation2d reef;
        switch (alliance) {
            default: assert false;
            case Blue: {
                reef = FieldConstants.kBlueReef;
                break;
            }
            case Red: {
                reef = FieldConstants.kRedReef;
                break;
            }
        }
        return reef;
    }

    private Translation2d getRobotToReef(Pose2d robotPose) {
        Translation2d robotToReef = m_reef.minus(robotPose.getTranslation());
        return robotToReef;
    }

    // Calculates the desired angle from the robot's current angle
    private Rotation2d getDesiredRotation(Pose2d robotPose) {
        Rotation2d reefToRobotAngle = getRobotToReef(robotPose).getAngle();
        Rotation2d desiredAngle = new Rotation2d(
            Math.floor((reefToRobotAngle.getRadians() + (Math.PI/6.0)) / (Math.PI/3.0)) * (Math.PI/3.0) + Math.PI);
        return desiredAngle;
    }

    public Rotation2d getRotationDeviation(Pose2d robotPose) {
        Rotation2d currentRotation = robotPose.getRotation();
        Rotation2d desiredRotation = getDesiredRotation(robotPose);
        Rotation2d rotationDeviation = currentRotation.minus(desiredRotation);
        return rotationDeviation;
    }
}
