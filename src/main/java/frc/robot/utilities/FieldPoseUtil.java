
package frc.robot.utilities;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.FieldConstants;

public class FieldPoseUtil {
	AprilTagFieldLayout m_field;
	Alliance m_alliance;
	double m_distanceFromWall;

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

	public FieldPoseUtil() {
		m_alliance = getAlliance();

		m_field = FieldConstants.kAprilTagFieldLayout;
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

	private Pose2d calculateTargetPoseAtStation(int aprilTagID, CoralStationSubPose subpose) {
		Pose2d stationPose = m_field.getTagPose(aprilTagID).get().toPose2d();
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

	private Pose2d calculateTargetPoseAtReef(int aprilTagID, ReefSubPose subpose) {
		Pose2d stationPose = m_field.getTagPose(aprilTagID).get().toPose2d();
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
				if (m_alliance == Alliance.Blue) {
					return calculateTargetPoseAtStation(13, slot);
				} else {
					return calculateTargetPoseAtStation(1, slot);
				}
			case RIGHT:
				if (m_alliance == Alliance.Blue) {
					return calculateTargetPoseAtStation(12, slot);
				} else {
					return calculateTargetPoseAtStation(2, slot);
				}
			default:
				assert(false);
				return null;
		}
	}

	public Pose2d getTargetPoseAtProcessor() {
		Pose2d aprilTagPose;
		if (m_alliance == Alliance.Blue) {
			aprilTagPose = m_field.getTagPose(16).get().toPose2d();
		} else {
			aprilTagPose = m_field.getTagPose(3).get().toPose2d();
		}
		return offsetAprilTagPoseByRobot(aprilTagPose, AutoConstants.kWallOffset, AutoConstants.kProcessorParallelOffset);
	}

	public Pose2d getTargetPoseAtReef(ReefPose reefTime, ReefSubPose subpose) {
		switch (reefTime) {
			case TWELVE:
				if (m_alliance == Alliance.Blue) {
					return calculateTargetPoseAtReef(21, subpose);
				} else {
					return calculateTargetPoseAtReef(10, subpose);
				}
			case TWO:
				if (m_alliance == Alliance.Blue) {
					return calculateTargetPoseAtReef(22, subpose);
				} else {
					return calculateTargetPoseAtReef(9, subpose);
				}
			case FOUR:
				if (m_alliance == Alliance.Blue) {
					return calculateTargetPoseAtReef(17, subpose);
				} else {
					return calculateTargetPoseAtReef(8, subpose);
				}
			case SIX:
				if (m_alliance == Alliance.Blue) {
					return calculateTargetPoseAtReef(18, subpose);
				} else {
					return calculateTargetPoseAtReef(7, subpose);
				}
			case EIGHT:
				if (m_alliance == Alliance.Blue) {
					return calculateTargetPoseAtReef(19, subpose);
				} else {
					return calculateTargetPoseAtReef(6, subpose);
				}
			case TEN:
				if (m_alliance == Alliance.Blue) {
					return calculateTargetPoseAtReef(20, subpose);
				} else {
					return calculateTargetPoseAtReef(11, subpose);
				}
			default:
				assert(false);
				return null;
		}
	}
}
