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
import frc.robot.utilities.FieldPoseUtil.CoralStationSubPose;

public class GetCoral extends SequentialCommandGroup {
  private final DriveSubsystem m_drive;
  private final HandlerSubsystem m_handler;
  private final Crane m_crane;
  private final FieldPoseUtil m_fieldPoseUtil = new FieldPoseUtil();
  private final Translation2d m_intakePos =
    new Translation2d(CraneConstants.kIntakeAngle, CraneConstants.kIntakeHeight);
  private final Translation2d m_homePos =
    new Translation2d(CraneConstants.kPivotHome, CraneConstants.kElevatorHome);

  public GetCoral(DriveSubsystem drive, HandlerSubsystem handler, Crane crane,
      CoralStationSubPose subPose) {
    m_drive = drive;
    m_handler = handler;
    m_crane = crane;
    addRequirements(m_drive, m_handler, m_crane);

    Command driveToPose = new DriveToPose(
      m_fieldPoseUtil.getTargetPoseAtStation(m_fieldPoseUtil.closestStation(m_drive.getPose()), subPose),
      drive);

    addCommands(
      driveToPose,
      Commands.runOnce(() -> m_crane.moveTo(m_intakePos)),
      Commands.runOnce(() -> m_handler.intakeCoral()),
      Commands.waitUntil(() -> m_handler.isLoadedCoral()),
      Commands.runOnce(() -> m_crane.moveTo(m_homePos)),
      Commands.waitUntil(() -> m_crane.atSetpoint().isPresent())
    );
  }
}