package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DriveCommandConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utilities.PIDF;
import frc.robot.utilities.TunablePIDF;

/** Drive directly to a given pose. */
public class DriveToPose extends Command {
  private final Pose2d m_pose;
  private final DriveSubsystem m_drive;

  private static final TunablePIDF translatingPIDF = new TunablePIDF("DriveToPose.translatingPIDF",
    DriveCommandConstants.kTranslatingPIDF);
  private static final TunablePIDF turningPIDF = new TunablePIDF("DriveToPose.turningPIDF",
    DriveCommandConstants.kTurningPIDF);

  // Although it would appear that the max x,y velocity/acceleration values are too high because
  // the combined velocity/acceleration vectors exceed the limits, the drive subsystem
  // proportionally clamps out-of-range values. This means that we can take maximum advantage of
  // the drivetrain's actual limits rather than multiplying these limits by ~0.707 (1/sqrt(2)).
  // This would be capable of allowing us to overshoot and run into things, except that the
  // drivetrain supports sufficiently higher deceleration than acceleration that the x,y PID
  // controllers never generate clamped values when decelerating.
  private ProfiledPIDController m_xController = new ProfiledPIDController(
    translatingPIDF.get().p(),
    translatingPIDF.get().i(),
    translatingPIDF.get().d(),
    new TrapezoidProfile.Constraints(
      DriveConstants.kMaxSpeedMetersPerSecond,
      DriveConstants.kMaxAccelerationMetersPerSecondSquared),
    Constants.kDt);
  private ProfiledPIDController m_yController = new ProfiledPIDController(
    translatingPIDF.get().p(),
    translatingPIDF.get().i(),
    translatingPIDF.get().d(),
    new TrapezoidProfile.Constraints(
      DriveConstants.kMaxSpeedMetersPerSecond,
      DriveConstants.kMaxAccelerationMetersPerSecondSquared),
    Constants.kDt);
  private ProfiledPIDController m_angleController = new ProfiledPIDController(
    turningPIDF.get().p(),
    turningPIDF.get().i(),
    turningPIDF.get().d(),
    new TrapezoidProfile.Constraints(
      DriveConstants.kMaxAngularSpeedRadiansPerSecond,
      DriveConstants.kMaxAngularAccelerationRadiansPerSecondSquared),
    Constants.kDt);

  public DriveToPose(Pose2d pose, DriveSubsystem drive) {
    m_pose = pose;
    m_drive = drive;
  
    addRequirements(m_drive);
  }

  private Translation2d getDesiredTranslation(Pose2d robotPose) {
    return m_pose.getTranslation();
  }

  private Translation2d getTranslationDeviation(Pose2d robotPose) {
    return getDesiredTranslation(robotPose).minus(robotPose.getTranslation());
  }

  private Rotation2d getDesiredRotation(Pose2d robotPose) {
    return m_pose.getRotation();
  }

  private Rotation2d getRotationDeviation(Pose2d robotPose) {
    Rotation2d currentRotation = robotPose.getRotation();
    Rotation2d desiredRotation = getDesiredRotation(robotPose);
    Rotation2d rotationDeviation = currentRotation.minus(desiredRotation);
    return rotationDeviation;
  }

  @Override
  public void initialize() {
    Pose2d robotPose = m_drive.getPose();
    Translation2d translationDeviation = getTranslationDeviation(robotPose);
    Translation2d velocity = m_drive.getVelocity();
    m_xController.reset(
      translationDeviation.getX(),
      velocity.getX()
    );
    m_yController.reset(
      translationDeviation.getY(),
      velocity.getY()
    );
    m_angleController.reset(
      getRotationDeviation(robotPose).getRadians(),
      m_drive.getAngularVelocity()
    );
  }

  @Override
  public void execute() {
    Pose2d robotPose = m_drive.getPose();
    Translation2d translationDeviation = getTranslationDeviation(robotPose);
    Rotation2d rotationDeviation = getRotationDeviation(robotPose);

    updateConstants();

    double xVelocity = m_xController.calculate(translationDeviation.getX());
    double yVelocity = m_yController.calculate(translationDeviation.getY());
    double angleVelocity = m_angleController.calculate(rotationDeviation.getRadians());
    m_drive.drive(
      xVelocity,
      yVelocity,
      angleVelocity,
      true);
  }

  private void updateConstants() {
    if (translatingPIDF.hasChanged()) {
      PIDF pidf = translatingPIDF.get();
      m_xController.setPID(pidf.p(), pidf.i(), pidf.d());
    }
    if (turningPIDF.hasChanged()) {
      PIDF pidf = turningPIDF.get();
      m_angleController.setPID(pidf.p(), pidf.i(), pidf.d());
    }
  }
}