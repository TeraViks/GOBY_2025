package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HandlerSubsystem;

public class Strafe extends SequentialCommandGroup {
  private final DriveSubsystem m_drive;

  public Strafe(HandlerSubsystem handler, DriveSubsystem drive) {
    m_drive = drive;
    Transform2d right = new Transform2d(0.0, -0.05, new Rotation2d());
    Transform2d left = new Transform2d(0.0, 0.1, new Rotation2d());

    addCommands(
      new DriveToPose(m_drive.getPose().plus(right), m_drive),
      new DriveToPose(m_drive.getPose().plus(left), m_drive)
    );
  }
}