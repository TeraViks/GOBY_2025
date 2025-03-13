package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
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
  private final AlgaeHeight m_algaeHeight;

  public GetAlgae(DriveSubsystem drive, HandlerSubsystem handler, Crane crane, 
      FieldPoseUtil fieldPoseUtil) {
    m_drive = drive;
    m_handler = handler;
    m_crane = crane;
    m_fieldPoseUtil = fieldPoseUtil;
    m_algaeHeight = null;
    addRequirements(m_drive, m_handler, m_crane);

    ReefPose reefHour = m_fieldPoseUtil.closestReefHour(m_drive.getPose());

    Command driveToPose = new DriveToPose(
      m_fieldPoseUtil.getTargetPoseAtReef(reefHour, ReefSubPose.ALGAE),
      drive);

    addCommands(
      driveToPose,
      Commands.runOnce(() -> m_fieldPoseUtil.whichAlgaeHeight(reefHour)),
      Commands.runOnce(() -> m_crane.moveTo(whichElevatorHeight())),
      Commands.waitUntil(() -> m_crane.atSetpoint().isPresent()),
      Commands.runOnce(() -> m_handler.intakeAlgae()),
      Commands.waitUntil(() -> m_handler.isLoadedAlgae())
    );
  }
 
  private Translation2d whichElevatorHeight() {
    return (m_algaeHeight == AlgaeHeight.DOWN) ? CraneConstants.kPositionLoAlgae : CraneConstants.kPositionHiAlgae;
  }
}