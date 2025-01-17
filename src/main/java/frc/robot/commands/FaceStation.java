// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.StationConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utilities.FaceStationUtil;
import frc.robot.utilities.PIDF;
import frc.robot.utilities.TunablePIDF;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FaceStation extends Command {
  DriveSubsystem m_drive;
  Supplier<Double> m_xVelocitySupplier;
  Supplier<Double> m_yVelocitySupplier;
  FaceStationUtil m_station;

  private static final TunablePIDF targetTurningPIDF = new TunablePIDF("Station.turningPIDF", StationConstants.kTurningPIDF);

  private ProfiledPIDController m_thetaController = new ProfiledPIDController(
    targetTurningPIDF.get().p(),
    targetTurningPIDF.get().i(),
    targetTurningPIDF.get().d(),
    new TrapezoidProfile.Constraints(
      DriveConstants.kMaxAngularSpeedRadiansPerSecond,
      
      DriveConstants.kMaxAngularAccelerationRadiansPerSecondSquared
    ),
    Constants.kDt);
  /** Creates a new FaceStation. */
  public FaceStation(DriveSubsystem drive, Supplier<Double> xVelocitySupplier, Supplier<Double> yVelocitySupplier) {
    m_drive = drive;
    m_xVelocitySupplier = xVelocitySupplier;
    m_yVelocitySupplier = yVelocitySupplier;
    m_station = new FaceStationUtil();

    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d robotPose = m_drive.getPose();
    m_thetaController.reset(
      m_station.getRotationDeviation(robotPose).getRadians(),
      m_drive.getAngularVelocity()
    );
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d robotPose = m_drive.getPose();
    Rotation2d rotationDeviation = m_station.getRotationDeviation(robotPose);

    updateConstants();

    double thetaVelocity = m_thetaController.calculate(rotationDeviation.getRadians());

    m_drive.drive(
      m_xVelocitySupplier.get(),
      m_yVelocitySupplier.get(),
      thetaVelocity,
      true
    );
  }

  private void updateConstants() {
    if (targetTurningPIDF.hasChanged()) {
      PIDF pidf = targetTurningPIDF.get();
      m_thetaController.setPID(pidf.p(), pidf.i(), pidf.d());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
