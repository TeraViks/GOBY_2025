package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.CraneConstants;
import frc.robot.subsystems.Crane;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HandlerSubsystem;
import frc.robot.utilities.FieldPoseUtil;
import frc.robot.utilities.FieldPoseUtil.ReefSubPose;

public class LevelOnePlacement extends SequentialCommandGroup {
  public LevelOnePlacement(DriveSubsystem drive, HandlerSubsystem handler, Crane crane,
    FieldPoseUtil fieldPoseUtil, ReefSubPose subPose) {
    addRequirements(drive, handler, crane);

    addCommands(
      Commands.runOnce(() -> crane.moveTo(CraneConstants.kPositionL1a)),
      Commands.defer((() -> {
        return new DriveToPose(
          fieldPoseUtil.getTargetPoseAtReef(
            fieldPoseUtil.closestReefHour(drive.getPose()),
            subPose).plus(AutoConstants.kL1ExtraReefOffset),  
          drive);
      }), Set.of(drive)),

      Commands.waitUntil(() -> crane.atGoal().isPresent()),
      Commands.runOnce(() -> handler.slowEject()),
      Commands.waitUntil(() -> handler.isEmpty()),
      Commands.runOnce(() -> crane.moveTo(CraneConstants.kPositionL1b)),

      Commands.defer((() -> {
        Command driveToPose =
          new DriveToPose(drive.getPose().plus(AutoConstants.kCoralL1PlacementMove), drive);
        return driveToPose;
      }), Set.of(drive))
    );
  }
}