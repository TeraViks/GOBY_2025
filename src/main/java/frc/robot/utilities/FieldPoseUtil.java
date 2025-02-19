package frc.robot.utilities;

import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.FieldConstants;

public class FieldPoseUtil {
  public record AprilTags(
    AprilTag reefTwelve,
    AprilTag reefTwo,
    AprilTag reefFour,
    AprilTag reefSix,
    AprilTag reefEight,
    AprilTag reefTen,
    AprilTag coralStationLeft,
    AprilTag coralStationRight,
    AprilTag processor,
    AprilTag bargeLeft,
    AprilTag bargeRight
  ) {
    private Translation2d reefCenter() {
      // The two AprilTags used have to be across from each other.
      Translation2d trans1 = reefTwelve.pose.getTranslation().toTranslation2d();
      Translation2d trans2 = reefSix.pose.getTranslation().toTranslation2d();
      return new Translation2d(
        (trans1.getX() + trans2.getX()) / 2.0,
        (trans1.getY() + trans2.getY()) / 2.0
      );
    }

    private ArrayList<Pose2d> coralStations() {
      return new ArrayList<>() {{
        add(coralStationLeft.pose.toPose2d());
        add(coralStationRight.pose.toPose2d());
      }};
    }
  }

  enum ReefPose {
    TWELVE,
    TWO,
    FOUR,
    SIX,
    EIGHT,
    TEN
  }

  enum ReefSubPose {
    A,
    B
  }

  enum CoralStationPose {
    LEFT,
    RIGHT
  }

  enum CoralStationSubPose {
    ONE,
    TWO,
    THREE,
    FOUR
  }

  Alliance m_alliance;
  AprilTags m_aprilTags;
  Translation2d m_reefCenter;
  ArrayList<Pose2d> m_coralStations;

  public FieldPoseUtil() {
    m_alliance = getAlliance();
    switch (m_alliance) {
      case Blue:
        m_aprilTags = FieldConstants.kBlueAprilTags;
        break;
      case Red:
        m_aprilTags = FieldConstants.kRedAprilTags;
        break;
    }
    m_reefCenter = m_aprilTags.reefCenter();
    m_coralStations = m_aprilTags.coralStations();
  }

  private static Alliance getAlliance() {
    Optional<Alliance> allianceOpt = DriverStation.getAlliance();
    Alliance alliance = allianceOpt.isPresent() ? allianceOpt.get() : Alliance.Blue;
    return alliance;
  }

  private Pose2d offsetAprilTagPoseByRobot(Pose2d aprilTagPose, double wallOffset, double parallelOffset) {
    double distanceFromWall = AutoConstants.kBumperToRobotCenter + wallOffset;
    double aprilTagRotationRadians = aprilTagPose.getRotation().getRadians();

    double yTranslation = distanceFromWall * Math.sin(aprilTagRotationRadians);
    double xTranslation = distanceFromWall * Math.cos(aprilTagRotationRadians);

    Transform2d transform = new Transform2d(xTranslation, yTranslation, new Rotation2d(Math.PI));

    Pose2d robotPoseFacingAprilTag = aprilTagPose.plus(transform);
    return offsetPoseParallel(robotPoseFacingAprilTag, parallelOffset);
  }

  private Pose2d offsetPoseParallel(Pose2d pose, double parallelOffset) {
    double poseRot = pose.getRotation().getRadians();
    double yTranslation = parallelOffset * Math.sin(poseRot);
    double xTranslation = parallelOffset * Math.cos(poseRot);

    Transform2d transform = new Transform2d(xTranslation, yTranslation, new Rotation2d());

    return pose.plus(transform);
  }

  private Pose2d calculateTargetPoseAtStation(AprilTag aprilTag, CoralStationSubPose subpose) {
    Pose2d stationPose = aprilTag.pose.toPose2d();
    double parallelOffset;
    switch (subpose) {
      case ONE:
        parallelOffset = AutoConstants.kStationParallelOffset1;
        break;
      case TWO:
        parallelOffset = AutoConstants.kStationParallelOffset2;
        break;
      case THREE:
        parallelOffset = AutoConstants.kStationParallelOffset3;
        break;
      case FOUR:
        parallelOffset = AutoConstants.kStationParallelOffset4;
        break;
      default:
        parallelOffset = 0;
        break;
    }
    return offsetAprilTagPoseByRobot(stationPose, AutoConstants.kWallOffset, parallelOffset);
  }

  private Pose2d calculateTargetPoseAtReef(AprilTag aprilTag, ReefSubPose subpose) {
    Pose2d stationPose = aprilTag.pose.toPose2d();
    double parallelOffset;
    switch (subpose) {
      case A:
        parallelOffset = AutoConstants.kReefParallelOffsetA;
        break;
      case B:
        parallelOffset = AutoConstants.kReefParallelOffsetB;
        break;
      default:
        parallelOffset = 0.0;
    }
    return offsetAprilTagPoseByRobot(stationPose, AutoConstants.kWallOffset, parallelOffset);
  }

  public Pose2d getTargetPoseAtStation(CoralStationPose station, CoralStationSubPose slot) {
    switch (station) {
      case LEFT:
        return calculateTargetPoseAtStation(m_aprilTags.coralStationLeft, slot);
      case RIGHT:
        return calculateTargetPoseAtStation(m_aprilTags.coralStationRight, slot);
      default:
        assert(false);
        return null;
    }
  }

  public Pose2d getTargetPoseAtProcessor() {
    Pose2d aprilTagPose = m_aprilTags.processor.pose.toPose2d();
    return offsetAprilTagPoseByRobot(aprilTagPose, AutoConstants.kWallOffset,
      AutoConstants.kProcessorParallelOffset);
  }

  public Pose2d getTargetPoseAtReef(ReefPose reefTime, ReefSubPose subpose) {
    switch (reefTime) {
      case TWELVE:
        return calculateTargetPoseAtReef(m_aprilTags.reefTwelve, subpose);
      case TWO:
        return calculateTargetPoseAtReef(m_aprilTags.reefTwo, subpose);
      case FOUR:
        return calculateTargetPoseAtReef(m_aprilTags.reefFour, subpose);
      case SIX:
        return calculateTargetPoseAtReef(m_aprilTags.reefSix, subpose);
      case EIGHT:
        return calculateTargetPoseAtReef(m_aprilTags.reefEight, subpose);
      case TEN:
        return calculateTargetPoseAtReef(m_aprilTags.reefTen, subpose);
      default:
        assert(false);
        return null;
    }
  }

  public Translation2d getReefCenter() {
    return m_reefCenter;
  }

  public ArrayList<Pose2d> getCoralStations() {
    return m_coralStations;
  }
}
