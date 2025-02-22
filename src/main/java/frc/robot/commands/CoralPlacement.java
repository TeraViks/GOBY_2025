package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Crane;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HandlerSubsystem;
import frc.robot.utilities.FieldPoseUtil;
import frc.robot.utilities.FieldPoseUtil.ReefSubPose;

public class CoralPlacement extends SequentialCommandGroup {
  private final DriveSubsystem m_drive;
  private final HandlerSubsystem m_handler;
  private final Crane m_crane;
  private FieldPoseUtil m_fieldPoseUtil = new FieldPoseUtil();

  public CoralPlacement(DriveSubsystem drive, HandlerSubsystem handler, Crane crane, ReefSubPose subPose, Translation2d cranePosition) {
    m_drive = drive;
    m_handler = handler;
    m_crane = crane;
    addRequirements(m_drive, m_handler, m_crane);

    Command driveToPose = new DriveToPose(m_fieldPoseUtil.getTargetPoseAtReef(m_fieldPoseUtil.closestReefTime(m_drive.getPose()), subPose), drive);

    addCommands(
      driveToPose,
      //TODO: strafe and align
      Commands.runOnce(() -> m_crane.moveTo(cranePosition)),
      Commands.waitUntil(() -> m_crane.atSetpoint().isPresent()),
      Commands.runOnce(() -> m_handler.eject()),
      Commands.waitUntil(() -> !m_handler.isLoadedCoral())
    );
  }
}
