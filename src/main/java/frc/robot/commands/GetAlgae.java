package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.CraneConstants;
import frc.robot.subsystems.Crane;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HandlerSubsystem;
import frc.robot.utilities.FieldPoseUtil;
import frc.robot.utilities.FieldPoseUtil.AlgaeHeight;
import frc.robot.utilities.FieldPoseUtil.ReefPose;
import frc.robot.utilities.FieldPoseUtil.ReefSubPose;

public class GetAlgae extends SequentialCommandGroup {
  private final DriveSubsystem m_drive;
  private final HandlerSubsystem m_handler;
  private final Crane m_crane;
  private final FieldPoseUtil m_fieldPoseUtil;
  private AlgaeHeight m_algaeHeight;

  public GetAlgae(DriveSubsystem drive, HandlerSubsystem handler, Crane crane, 
      FieldPoseUtil fieldPoseUtil) {
    m_drive = drive;
    m_handler = handler;
    m_crane = crane;
    m_fieldPoseUtil = fieldPoseUtil;
    m_algaeHeight = null;
    addRequirements(m_drive, m_handler, m_crane);

    addCommands(
      Commands.defer((() -> {
        Pose2d pose = m_drive.getPose();
        ReefPose reefHour = m_fieldPoseUtil.closestReefHour(pose);
        m_algaeHeight = m_fieldPoseUtil.whichAlgaeHeight(reefHour);
        Command driveToPose = new DriveToPose(
          m_fieldPoseUtil.getTargetPoseAtReef(m_fieldPoseUtil.closestReefHour(pose), 
          ReefSubPose.ALGAE),
          drive);
        return driveToPose;
      }), Set.of(drive)),

      Commands.runOnce(() -> m_crane.moveTo(whichElevatorHeight(m_algaeHeight))),
      Commands.waitUntil(() -> m_crane.atGoal().isPresent()),

      Commands.runOnce(() -> m_handler.intakeAlgae()),
      Commands.defer((() -> {
        Transform2d move = new Transform2d(AutoConstants.kAlgaeIntakePositionTwo, 0.0, new Rotation2d());
        Command driveToPose = new DriveToPose(m_drive.getPose().plus(move), m_drive);
        return driveToPose;
      }), Set.of(drive)),
      Commands.waitUntil(() -> m_handler.isLoadedAlgae())
    );
  }
 
  private Translation2d whichElevatorHeight(AlgaeHeight algaeHeight) {
    return (algaeHeight == AlgaeHeight.DOWN)
      ? CraneConstants.kPositionLoAlgae
      : CraneConstants.kPositionHiAlgae;
  }
}