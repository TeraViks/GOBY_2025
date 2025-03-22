package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
  private final FieldPoseUtil m_fieldPoseUtil;
  private final Crane m_crane;
  private AlgaeHeight m_algaeHeight;
  private ReefPose m_reefHour;

  public GetAlgae(DriveSubsystem drive, HandlerSubsystem handler, Crane crane, 
      FieldPoseUtil fieldPoseUtil) {
    m_drive = drive;
    m_handler = handler;
    m_crane = crane;
    m_fieldPoseUtil = fieldPoseUtil;
    m_algaeHeight = null;
    m_reefHour = null;
    addRequirements(m_drive, m_handler, m_crane);

    addCommands(
      Commands.defer((() -> {return new DriveToPose(
        m_fieldPoseUtil.getTargetPoseAtReef(m_fieldPoseUtil.closestReefHour(m_drive.getPose()), ReefSubPose.ALGAE),
        drive);}), Set.of(drive)),

      Commands.defer((() -> {return Commands.runOnce(() -> m_reefHour = m_fieldPoseUtil.closestReefHour(m_drive.getPose()));}), Set.of()),
      Commands.defer((() -> {return Commands.runOnce(() -> m_algaeHeight = m_fieldPoseUtil.whichAlgaeHeight(m_reefHour));}), Set.of()),

      Commands.defer((() -> {return Commands.runOnce(() -> m_crane.moveTo(whichElevatorHeight(m_algaeHeight)));}), Set.of(crane)),
      Commands.waitUntil(() -> m_crane.atGoal().isPresent()),

      Commands.runOnce(() -> m_handler.intakeAlgae()),
      Commands.waitUntil(() -> m_handler.isLoadedAlgae())
    );
  }
 
  private Translation2d whichElevatorHeight(AlgaeHeight algaeHeight) {
    return (algaeHeight == AlgaeHeight.DOWN) ? CraneConstants.kPositionLoAlgae : CraneConstants.kPositionHiAlgae;
  }
}