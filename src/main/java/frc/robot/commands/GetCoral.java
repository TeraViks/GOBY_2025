package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
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

    Pose2d robotPose = m_drive.getPose();
    Pose2d closestStation = m_station.closestStation(robotPose);
    //TODO: where should the robot be offset from the station

    //TODO: Create the constraints to use while pathfinding
    PathConstraints constraints = new PathConstraints(
      3.0, 4.0,
      Units.degreesToRadians(540), Units.degreesToRadians(720));

    Command pathfindingCommand = AutoBuilder.pathfindToPose(
      closestStation,
      constraints,
      0.0
    );

    Command faceStation = new FaceStation(drive, null, null);

    addCommands(
      pathfindingCommand,
      faceStation, 
      //TODO: aligning command
      Commands.runOnce(() -> m_handler.intake()),
      Commands.waitUntil(() -> m_handler.isLoaded())
    );
  }
}
