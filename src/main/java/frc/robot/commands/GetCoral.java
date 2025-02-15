package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HandlerSubsystem;
import frc.robot.utilities.FaceStationUtil;

public class GetCoral extends SequentialCommandGroup {
  private final DriveSubsystem m_drive;
  private final FaceStationUtil m_station;
  private final HandlerSubsystem m_handler;

  public GetCoral(DriveSubsystem drive, HandlerSubsystem handler) {
    m_drive = drive;
    m_station = new FaceStationUtil();
    m_handler = handler;
    addRequirements(m_drive, m_handler);

    Command faceStation = new FaceStation(drive, null, null);
    Command driveToPose = new DriveToPose(null, drive); //TODO: get poses

    addCommands(
      faceStation,
      driveToPose,
      //TODO: pivot and elevator (from crane subsystem)
      Commands.runOnce(() -> m_handler.intakeCoral()),
      Commands.waitUntil(() -> m_handler.isCoralLoaded())
    );
  }
}
