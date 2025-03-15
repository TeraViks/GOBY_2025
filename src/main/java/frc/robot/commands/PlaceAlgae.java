// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.CraneConstants;
import frc.robot.subsystems.Crane;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HandlerSubsystem;
import frc.robot.utilities.FieldPoseUtil;

public class PlaceAlgae extends SequentialCommandGroup {
  private final DriveSubsystem m_drive;
  private final HandlerSubsystem m_handler;
  private final Crane m_crane;
  private final FieldPoseUtil m_fieldPoseUtil;

  public PlaceAlgae(DriveSubsystem drive, HandlerSubsystem handler, Crane crane, FieldPoseUtil fieldPoseUtil) {
    m_drive = drive;
    m_handler = handler;
    m_crane = crane;
    m_fieldPoseUtil = fieldPoseUtil;
    addRequirements(m_drive, m_handler, m_crane);
    DriveToPose driveToPose = new DriveToPose(m_fieldPoseUtil.getTargetPoseAtProcessor(), m_drive);

    addCommands(
      driveToPose,
      Commands.runOnce(() -> m_crane.moveTo(CraneConstants.kPositionProcessor), m_crane),
      Commands.waitUntil(() -> m_crane.atSetpoint().isPresent()),
      Commands.runOnce(() -> m_handler.eject(), m_handler),
      Commands.waitUntil(() -> m_handler.isEmpty()),
      Commands.runOnce(() -> m_crane.moveTo(CraneConstants.kPositionHome), m_crane)
    );
  }
}
