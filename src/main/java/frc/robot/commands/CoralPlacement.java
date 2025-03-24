package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.CraneConstants;
import frc.robot.subsystems.Crane;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HandlerSubsystem;
import frc.robot.utilities.FieldPoseUtil;
import frc.robot.utilities.FieldPoseUtil.ReefSubPose;

public class CoralPlacement extends SequentialCommandGroup {
  public CoralPlacement(DriveSubsystem drive, HandlerSubsystem handler, Crane crane,
      FieldPoseUtil fieldPoseUtil, ReefSubPose subPose, Translation2d cranePosition) {
    addRequirements(drive, handler, crane);

    addCommands(
      Commands.parallel(
        Commands.defer((() -> {
          return new DriveToPose(
            fieldPoseUtil.getTargetPoseAtReef(
              fieldPoseUtil.closestReefHour(drive.getPose()),
              subPose),
            drive);
        }), Set.of(drive)),
        Commands.runOnce(() -> crane.moveTo(cranePosition), crane)
      ),
      Commands.waitUntil(() -> crane.atGoal().isPresent()),
      Commands.runOnce(() -> handler.eject(), handler),
      Commands.waitUntil(() -> handler.isEmpty()),
    );
  }
}