package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Crane;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HandlerSubsystem;
import frc.robot.utilities.FieldPoseUtil;
import frc.robot.utilities.FieldPoseUtil.ReefSubPose;

public class CoralPlacement extends SequentialCommandGroup {
  private final HandlerSubsystem m_handler;
  private final DriveSubsystem m_drive;
  private final Crane m_crane;
  private final FieldPoseUtil m_fieldPoseUtil;

  public CoralPlacement(DriveSubsystem drive, HandlerSubsystem handler, Crane crane, 
      FieldPoseUtil fieldPoseUtil, ReefSubPose subPose, Translation2d cranePosition) {
    m_handler = handler;
    m_drive = drive;
    m_crane = crane;
    m_fieldPoseUtil = fieldPoseUtil;
    addRequirements(m_drive, m_handler, m_crane);
    
    addCommands(
      Commands.defer((() -> {return new DriveToPose(
        m_fieldPoseUtil.getTargetPoseAtReef(m_fieldPoseUtil.closestReefHour(m_drive.getPose()), subPose),
        drive);}), Set.of(drive)),
      Commands.runOnce(() -> m_crane.moveTo(cranePosition)),
      Commands.waitUntil(() -> m_crane.atGoal().isPresent()),
      Commands.runOnce(() -> m_handler.eject()),
      Commands.waitUntil(() -> !m_handler.isLoadedCoral())
    );
  }
}