package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Crane;
import frc.robot.subsystems.HandlerSubsystem;
import frc.robot.utilities.FieldPoseUtil.ReefSubPose;

public class CoralPlacement extends SequentialCommandGroup {
  private final HandlerSubsystem m_handler;
  private final Crane m_crane;

  public CoralPlacement(HandlerSubsystem handler, Crane crane, ReefSubPose subPose, Translation2d cranePosition) {
    m_handler = handler;
    m_crane = crane;
    addRequirements(m_handler, m_crane);

    addCommands(
      Commands.runOnce(() -> m_crane.moveTo(cranePosition)),
      Commands.waitUntil(() -> m_crane.atSetpoint().isPresent())
    );
  }
}