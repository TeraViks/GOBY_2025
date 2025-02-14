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
import frc.robot.Constants;
import frc.robot.Constants.DriveCommandConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utilities.FaceStationUtil;
import frc.robot.utilities.PIDF;
import frc.robot.utilities.TunablePIDF;

public class FaceStation extends Command {
  DriveSubsystem m_drive;
  Supplier<Double> m_xVelocitySupplier;
  Supplier<Double> m_yVelocitySupplier;
  FaceStationUtil m_station;

  private static final TunablePIDF turningPIDF = new TunablePIDF("Station.turningPIDF",
    DriveCommandConstants.kTurningPIDF);

  private ProfiledPIDController m_angleController = new ProfiledPIDController(
    turningPIDF.get().p(),
    turningPIDF.get().i(),
    turningPIDF.get().d(),
    new TrapezoidProfile.Constraints(
      DriveConstants.kMaxAngularSpeedRadiansPerSecond,
      DriveConstants.kMaxAngularAccelerationRadiansPerSecondSquared
    ),
    Constants.kDt);

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
    m_angleController.reset(
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

    double angleVelocity = m_angleController.calculate(rotationDeviation.getRadians() + Math.PI);

    m_drive.drive(
      m_xVelocitySupplier.get(),
      m_yVelocitySupplier.get(),
      angleVelocity,
      true
    );
  }

  private void updateConstants() {
    if (turningPIDF.hasChanged()) {
      PIDF pidf = turningPIDF.get();
      m_angleController.setPID(pidf.p(), pidf.i(), pidf.d());
    }
  }
}
