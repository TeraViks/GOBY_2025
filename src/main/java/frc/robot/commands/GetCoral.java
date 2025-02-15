package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HandlerSubsystem;

public class GetCoral extends SequentialCommandGroup {
  private final DriveSubsystem m_drive;
  private final HandlerSubsystem m_handler;

  public GetCoral(DriveSubsystem drive, HandlerSubsystem handler) {
    m_drive = drive;
    m_handler = handler;
    addRequirements(m_drive, m_handler);

    Command driveToPose = new DriveToPose(null, drive); //TODO: get poses

    addCommands(
      driveToPose,
      //TODO: pivot and elevator (from crane subsystem)
      Commands.runOnce(() -> m_handler.intakeCoral()),
      Commands.waitUntil(() -> m_handler.isLoadedCoral())
    );
  }
}
