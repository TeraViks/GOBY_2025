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

public class FaceStationUtil {
    private final Translation2d m_coralStation1;
    private final Translation2d m_coralStation2;

    private Translation2d m_station;
    
    public FaceStationUtil() {
        Alliance alliance = getAlliance();

        m_coralStation1 = getStation(alliance, 1);
        m_coralStation2 = getStation(alliance, 2);
    }

    private static Alliance getAlliance() {
        Optional<Alliance> allianceOpt = DriverStation.getAlliance();
        Alliance alliance = allianceOpt.isPresent() ? allianceOpt.get() : Alliance.Blue;
        return alliance;
    }

    private static Translation2d getStation(Alliance alliance, int stationNumber) {
        Translation2d coralStation;
        switch (alliance) {
            default: assert false;
            case Blue: { 
                coralStation = stationNumber == 1 ? FieldConstants.kBlueCoralStation1 : FieldConstants.kBlueCoralStation2;
                break;
            }
            case Red: {
                coralStation = stationNumber == 1 ? FieldConstants.kRedCoralStation1 : FieldConstants.kBlueCoralStation2;
                break;
            }
        }
        return coralStation;
    }

    private static Translation2d getRobotToCoralStation(Pose2d robotPose, Translation2d station) {
        Translation2d robotToStation = station.minus(robotPose.getTranslation());
        return robotToStation;
    }

    private void updateClosestStation(Pose2d robotPose) {
        Translation2d robotPos = robotPose.getTranslation();
        if (m_coralStation1.getDistance(robotPos) < m_coralStation2.getDistance(robotPos)) {
            m_station = m_coralStation1;
        } else {
            m_station = m_coralStation2;
        }
    }

    public Rotation2d getRotationDeviation(Pose2d robotPose) {
        updateClosestStation(robotPose);
        Rotation2d currentRotation = robotPose.getRotation();
        Rotation2d desiredRotation = getRobotToCoralStation(robotPose, m_station).getAngle();
        Rotation2d rotationDeviation = currentRotation.minus(desiredRotation);
        return rotationDeviation;
    }

    public double getStationDistance(Pose2d robotPose) {
        updateClosestStation(robotPose);
        Translation2d robotToStation = getRobotToCoralStation(robotPose, m_station);
        double stationDistance = robotToStation.getNorm();
        return stationDistance;
    }
}
