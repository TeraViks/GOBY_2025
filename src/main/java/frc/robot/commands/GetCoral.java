package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HandlerSubsystem;
import frc.robot.utilities.FaceStationUtil;
import frc.robot.utilities.FieldPoseUtil;
import frc.robot.utilities.FieldPoseUtil.CoralStationPose;
import frc.robot.utilities.FieldPoseUtil.CoralStationSubPose;

public class GetCoral extends SequentialCommandGroup {
  private final DriveSubsystem m_drive;
  private final HandlerSubsystem m_handler;
  private final FaceStationUtil m_faceStationUtil;
  private final FieldPoseUtil m_fieldPoseUtil;

  public GetCoral(DriveSubsystem drive, HandlerSubsystem handler) { //TODO: Also include crane in the constructor and requirements
    m_drive = drive;
    m_handler = handler;
    addRequirements(m_drive, m_handler);
    m_faceStationUtil = new FaceStationUtil();
    m_fieldPoseUtil = new FieldPoseUtil();
    CoralStationPose station = m_faceStationUtil.closestStationEnum(m_drive.getPose());
    Pose2d desiredPose = m_fieldPoseUtil.getTargetPoseAtStation(station, CoralStationSubPose.ONE); //TODO: How do we decide which slot to go to? operator decides?
    Command driveToPose = new DriveToPose(desiredPose, drive);

    addCommands(
      driveToPose,
      //TODO: pivot and elevator (from crane subsystem)
      Commands.runOnce(() -> m_handler.intakeCoral()),
      Commands.waitUntil(() -> m_handler.isLoadedCoral())
    );
  }
}
