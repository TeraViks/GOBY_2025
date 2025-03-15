package frc.robot.utilities;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.Map;
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

  public enum ReefPose {
    TWELVE,
    TWO,
    FOUR,
    SIX,
    EIGHT,
    TEN;

    private AprilTag mapToAprilTag(AprilTags aprilTags) {
      switch (this) {
        default: assert(false);
        case TWELVE: return aprilTags.reefTwelve;
        case TWO: return aprilTags.reefTwo;
        case FOUR: return aprilTags.reefFour;
        case SIX: return aprilTags.reefSix;
        case EIGHT: return aprilTags.reefEight;
        case TEN: return aprilTags.reefTen;
      }
    }
  }

  public enum AlgaeHeight {
    UP,
    DOWN
  }

  public AlgaeHeight whichAlgaeHeight(ReefPose reefPose) {
    switch(reefPose) {
      default: assert(false);
      case TWO: return AlgaeHeight.UP;
      case FOUR: return AlgaeHeight.DOWN;
      case SIX: return AlgaeHeight.UP;
      case EIGHT: return AlgaeHeight.DOWN;
      case TEN: return AlgaeHeight.UP;
      case TWELVE: return AlgaeHeight.DOWN;
    }
  }

  public enum ReefSubPose {
    A(1),
    B(-1),
    ALGAE(0);

    // Integer multiple of reef branch offset. Negative is right.
    int m_yBranchDelta;

    ReefSubPose(int yBranchDelta) {
      m_yBranchDelta = yBranchDelta;
    }

    private double getYOffset() {
      return (double)m_yBranchDelta * AutoConstants.kReefBranchOffset;
    }
  }

  public enum CoralStationPose {
    LEFT,
    RIGHT;

    private AprilTag mapToAprilTag(AprilTags aprilTags) {
      switch(this) {
        default: assert(false);
        case LEFT: return aprilTags.coralStationLeft;
        case RIGHT: return aprilTags.coralStationRight;
      }
    }
  }

  public enum CoralStationSubPose {
    ONE(4),
    TWO(3),
    THREE(2),
    FOUR(1),
    FIVE(0), // Center slot, aligned with station AprilTag.
    SIX(-1),
    SEVEN(-2),
    EIGHT(-3),
    NINE(-4);

    // Integer multiple of station slot spacing. Negative is right.
    int m_ySlotDelta;

    CoralStationSubPose(int ySlotDelta) {
      m_ySlotDelta = ySlotDelta;
    }

    private double getYOffset() {
      return (double)m_ySlotDelta * AutoConstants.kStationSlotSpacing;
    }
  }

  private Alliance m_alliance;
  private AprilTags m_aprilTags;
  private Translation2d m_reefCenter;
  private ArrayList<Pose2d> m_coralStations;
  private Map<AprilTag, ReefPose> m_reefMap;
  private Map<AprilTag, CoralStationPose> m_coralStationMap;

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
    m_reefMap = Map.of(
      m_aprilTags.reefTwo, ReefPose.TWO,
      m_aprilTags.reefFour, ReefPose.FOUR,
      m_aprilTags.reefSix, ReefPose.SIX,
      m_aprilTags.reefEight, ReefPose.EIGHT,
      m_aprilTags.reefTen, ReefPose.TEN,
      m_aprilTags.reefTwelve, ReefPose.TWELVE
    );
    m_coralStationMap = Map.of(
      m_aprilTags.coralStationLeft, CoralStationPose.LEFT,
      m_aprilTags.coralStationRight, CoralStationPose.RIGHT
    );
  }

  private static Alliance getAlliance() {
    Optional<Alliance> allianceOpt = DriverStation.getAlliance();
    Alliance alliance = allianceOpt.isPresent() ? allianceOpt.get() : Alliance.Blue;
    return alliance;
  }

  public Translation2d getReefCenter() {
    return m_reefCenter;
  }

  public ArrayList<Pose2d> getCoralStations() {
    return m_coralStations;
  }

  public ReefPose closestReefHour(Pose2d robotPose) {
    Translation2d robotPos = robotPose.getTranslation();
    AprilTag nearestReefHour = Collections.min(
      m_reefMap.keySet(),
      Comparator.comparing((AprilTag aprilTag) ->
        robotPos.getDistance(aprilTag.pose.toPose2d().getTranslation()))
    );
    return m_reefMap.get(nearestReefHour);
  }

  public CoralStationPose closestStation(Pose2d robotPose) {
    Translation2d robotPos = robotPose.getTranslation();
    AprilTag nearestStation = Collections.min(
      m_coralStationMap.keySet(),
      Comparator.comparing((AprilTag aprilTag) ->
        robotPos.getDistance(aprilTag.pose.toPose2d().getTranslation()))
    );
    return m_coralStationMap.get(nearestStation);
  }

  private static Pose2d rotateByPi(Pose2d pose) {
    return new Pose2d(pose.getTranslation(), pose.getRotation().plus(Rotation2d.kPi));
  }

  public Pose2d getTargetPoseAtReef(ReefPose reefTime, ReefSubPose subpose) {
    Pose2d reefPose = reefTime.mapToAprilTag(m_aprilTags).pose.toPose2d();
    double xOffset = AutoConstants.kReefXOffset;
    double yOffset = AutoConstants.kStingerYOffset + subpose.getYOffset();
    return rotateByPi(reefPose).plus(new Transform2d(xOffset, yOffset, Rotation2d.kZero));
  }

  public Pose2d getTargetPoseAtStation(CoralStationPose station, CoralStationSubPose slot) {
    Pose2d stationPose = station.mapToAprilTag(m_aprilTags).pose.toPose2d();
    double xOffset = AutoConstants.kStationXOffset;
    double yOffset = AutoConstants.kStingerYOffset + slot.getYOffset();
    return rotateByPi(stationPose).plus(new Transform2d(xOffset, yOffset, Rotation2d.kZero));
  }

  public Pose2d getTargetPoseAtProcessor() {
    Pose2d processorPose = m_aprilTags.processor.pose.toPose2d();
    double xOffset = AutoConstants.kProcessorXOffset;
    double yOffset = AutoConstants.kStingerYOffset;
    return rotateByPi(processorPose).plus(new Transform2d(xOffset, yOffset, Rotation2d.kZero));
  }
}