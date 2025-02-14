
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

	enum Pose {
		LEFT_STATION,
		RIGHT_STATION,
		PROCESSOR,
		TWO,
		FOUR,
		SIX,
		EIGHT,
		TEN,
		TWELVE
	}

	enum SubPose {
		A,
		B,
		ALGAE,
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

	private Pose2d calculateTargetPoseAtStation(int aprilTagID, SubPose subpose) {
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

	private Pose2d calculateTargetPoseAtReef(int aprilTagID, SubPose subpose) {
		Pose2d stationPose = m_field.getTagPose(aprilTagID).get().toPose2d();
		double parallelOffset;
		switch (subpose) {
			case A:
				parallelOffset = AutoConstants.kReefParallelOffsetA;
				break;
			case B:
				parallelOffset = AutoConstants.kReefParallelOffsetB;
				break;
			case ALGAE:
				parallelOffset = 0;
				break;
			default:
				parallelOffset = 0;
				break;
		}
		return offsetAprilTagPoseByRobot(stationPose, AutoConstants.kWallOffset, parallelOffset);
		 
	}

	public Pose2d getTargetPose(Pose pose, SubPose subpose) {
		Pose2d targetPose;
		switch (pose) {
			case LEFT_STATION:
				if (m_alliance == Alliance.Blue) {
					targetPose = calculateTargetPoseAtStation(13, subpose);
				} else {
					targetPose = calculateTargetPoseAtStation(1, subpose);
				}
				break;
			case RIGHT_STATION:
				if (m_alliance == Alliance.Blue) {
					targetPose = calculateTargetPoseAtStation(12, subpose);
				} else {
					targetPose = calculateTargetPoseAtStation(2, subpose);
				}
				break;
			case TWO:
				if (m_alliance == Alliance.Blue) {
					targetPose = calculateTargetPoseAtReef(22, subpose);
				} else {
					targetPose = calculateTargetPoseAtReef(9, subpose);
				}
				break;
			case FOUR:
				if (m_alliance == Alliance.Blue) {
					targetPose = calculateTargetPoseAtReef(17, subpose);
				} else {
					targetPose = calculateTargetPoseAtReef(8, subpose);
				}
				break;
			case SIX:
				if (m_alliance == Alliance.Blue) {
					targetPose = calculateTargetPoseAtReef(18, subpose);
				} else {
					targetPose = calculateTargetPoseAtReef(7, subpose);
				}
				break;
			case EIGHT:
				if (m_alliance == Alliance.Blue) {
					targetPose = calculateTargetPoseAtReef(19, subpose);
				} else {
					targetPose = calculateTargetPoseAtReef(6, subpose);
				}
				break;
			case TEN:
				if (m_alliance == Alliance.Blue) {
					targetPose = calculateTargetPoseAtReef(20, subpose);
				} else {
					targetPose = calculateTargetPoseAtReef(11, subpose);
				}
				break;
			case TWELVE:
				if (m_alliance == Alliance.Blue) {
					targetPose = calculateTargetPoseAtReef(21, subpose);
				} else {
					targetPose = calculateTargetPoseAtReef(10, subpose);
				}
				break;
			default:
				targetPose = null;
				break;
		}
		return targetPose;
	}
}
