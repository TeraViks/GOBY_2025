// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;
import frc.robot.utilities.FieldPoseUtil.CoralStationPose;

public class FaceStationUtil {
  private final ArrayList<Integer> m_coralStations;

  public FaceStationUtil() {
    Alliance alliance = getAlliance();
    m_coralStations = getStationsAprilTags(alliance);
  }

  private static Alliance getAlliance() {
    Optional<Alliance> allianceOpt = DriverStation.getAlliance();
    Alliance alliance = allianceOpt.isPresent() ? allianceOpt.get() : Alliance.Blue;
    return alliance;
  }

  private static ArrayList<Integer> getStationsAprilTags(Alliance alliance) {
    ArrayList<Integer> coralStations;
    switch (alliance) {
      default: assert false;
      case Blue: {
        coralStations = FieldConstants.kBlueCoralStationsAprilTags;
        break;
      }
      case Red: {
        coralStations = FieldConstants.kRedCoralStationsAprilTags;
        break;
      }
    }
    return coralStations;
  }

  private static Translation2d getRobotToCoralStation(Pose2d robotPose, Pose2d station) {
    Translation2d robotToStation = station.getTranslation().minus(robotPose.getTranslation());
    return robotToStation;
  }

  private Pose2d aprilTagToPose(int aprilTagID) {
    return FieldConstants.kAprilTagFieldLayout.getTagPose(aprilTagID).get().toPose2d();
  }

  private Integer closestStationImpl(Pose2d robotPose) {
    Translation2d robotPos = robotPose.getTranslation();
    int nearestStation = m_coralStations.get(0);
    for (int station : m_coralStations) {
      Pose2d stationPose = aprilTagToPose(station);
      if (stationPose.getTranslation().getDistance(robotPos) < aprilTagToPose(nearestStation).getTranslation().getDistance(robotPos)) {
        nearestStation = station;
      }
    }
    return nearestStation;
  }

  public CoralStationPose closestStationEnum(Pose2d robotPose) {
    return FieldPoseUtil.aprilTagIDToStationEnum(closestStationImpl(robotPose));
  }

  public Pose2d closestStationPose(Pose2d robotPose) {
    return aprilTagToPose(closestStationImpl(robotPose));
  }

  public Rotation2d getRotationDeviation(Pose2d robotPose) {
    Rotation2d currentRotation = robotPose.getRotation();
    Rotation2d desiredRotation = closestStationPose(robotPose).getRotation();
    Rotation2d rotationDeviation = currentRotation.minus(desiredRotation);
    return rotationDeviation;
  }

  public double getStationDistance(Pose2d robotPose) {
    Translation2d robotToStation = getRobotToCoralStation(robotPose, closestStationPose(robotPose));
    double stationDistance = robotToStation.getNorm();
    return stationDistance;
  }
}
