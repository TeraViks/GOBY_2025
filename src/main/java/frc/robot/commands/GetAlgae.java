package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.CraneConstants;
import frc.robot.subsystems.Crane;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HandlerSubsystem;
import frc.robot.utilities.FieldPoseUtil;
import frc.robot.utilities.FieldPoseUtil.AlgaeHeight;
import frc.robot.utilities.FieldPoseUtil.ReefPose;
import frc.robot.utilities.FieldPoseUtil.ReefSubPose;

public class GetAlgae extends SequentialCommandGroup {
  private Pose2d m_initialPose;
  private AlgaeHeight m_algaeHeight;

  public GetAlgae(DriveSubsystem drive, HandlerSubsystem handler, Crane crane,
      FieldPoseUtil fieldPoseUtil) {
    m_algaeHeight = null;
    addRequirements(drive, handler, crane);

    addCommands(
      Commands.defer(() -> {
        return Commands.runOnce(() -> {
          m_initialPose = drive.getPose();
          ReefPose reefHour = fieldPoseUtil.closestReefHour(m_initialPose);
          m_algaeHeight = fieldPoseUtil.whichAlgaeHeight(reefHour);
        });
      }, Set.of(drive)),
      Commands.runOnce(() -> crane.moveTo(whichElevatorHeight(m_algaeHeight)), crane),
      Commands.waitUntil(() -> crane.atGoal().isPresent()),
      Commands.defer((() -> {
        Command driveToPose = new DriveToPose(
          fieldPoseUtil.getTargetPoseAtReef(fieldPoseUtil.closestReefHour(m_initialPose),
          ReefSubPose.ALGAE),
          drive);
        return driveToPose;
      }), Set.of(drive)),
      Commands.runOnce(() -> handler.intakeAlgae(), handler),
      Commands.defer((() -> {
        Command driveToPose =
          new DriveToPose(drive.getPose().plus(AutoConstants.kAlgaeIntakeMove), drive);
        return driveToPose;
      }), Set.of(drive)),
      Commands.waitUntil(() -> handler.isLoadedAlgae())
    );
  }

  private Translation2d whichElevatorHeight(AlgaeHeight algaeHeight) {
    return (algaeHeight == AlgaeHeight.DOWN)
      ? CraneConstants.kPositionLoAlgae
      : CraneConstants.kPositionHiAlgae;
  }
}