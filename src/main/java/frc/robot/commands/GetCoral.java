package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.CraneConstants;
import frc.robot.subsystems.Crane;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HandlerSubsystem;
import frc.robot.subsystems.LightSubsystem;
import frc.robot.subsystems.LightSubsystem.Color;
import frc.robot.utilities.FieldPoseUtil;
import frc.robot.utilities.FieldPoseUtil.CoralStationSubPose;

public class GetCoral extends SequentialCommandGroup {
  private final DriveSubsystem m_drive;
  private final HandlerSubsystem m_handler;
  private final Crane m_crane;
  private final LightSubsystem m_lightSubsystem;
  private final FieldPoseUtil m_fieldPoseUtil;

  public GetCoral(DriveSubsystem drive, HandlerSubsystem handler, Crane crane, LightSubsystem lightSubsystem,
      FieldPoseUtil fieldPoseUtil, CoralStationSubPose subPose) {
    m_drive = drive;
    m_handler = handler;
    m_crane = crane;
    m_lightSubsystem = lightSubsystem;
    m_fieldPoseUtil = fieldPoseUtil;
    addRequirements(m_drive, m_handler, m_crane, m_lightSubsystem);

    Command driveToPose = new DriveToPose(
      m_fieldPoseUtil.getTargetPoseAtStation(m_fieldPoseUtil.closestStation(m_drive.getPose()), subPose),
      drive);

    addCommands(
      driveToPose,
      Commands.runOnce(() -> m_crane.moveTo(CraneConstants.kPositionIntake)),
      Commands.runOnce(() -> m_lightSubsystem.setColor(Color.GREEN)),
      Commands.runOnce(() -> m_handler.intakeCoral()),
      Commands.waitUntil(() -> m_handler.isLoadedCoral()),
      Commands.runOnce(() -> m_lightSubsystem.setColor(Color.OFF)),
      Commands.runOnce(() -> m_crane.moveTo(CraneConstants.kPositionHome)),
      Commands.waitUntil(() -> m_crane.atGoal().isPresent())
      .finallyDo(interrupted -> {
        if (interrupted) {
          m_lightSubsystem.setColor(Color.OFF);
        }
      })
    );
  }
}