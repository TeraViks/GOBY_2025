// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.revrobotics.spark.ClosedLoopSlot;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.CameraConstants;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  private final SwerveModule m_frontLeft = //Q1
      new SwerveModule(
          DriveConstants.kFrontLeftDriveMotorPort,
          DriveConstants.kFrontLeftTurningMotorPort,
          DriveConstants.kFrontLeftTurningEncoderPort,
          DriveConstants.kFrontLeftDriveReversed,
          DriveConstants.kFrontLeftTurningMotorReversed,
          DriveConstants.kFrontLeftEncoderReversed,
          DriveConstants.kFrontLeftEncoderOffset);

  private final SwerveModule m_rearLeft = //Q2
      new SwerveModule(
          DriveConstants.kRearLeftDriveMotorPort,
          DriveConstants.kRearLeftTurningMotorPort,
          DriveConstants.kRearLeftTurningEncoderPorts,
          DriveConstants.kRearLeftDriveReversed,
          DriveConstants.kRearLeftTurningMotorReversed,
          DriveConstants.kRearLeftEncoderReversed,
          DriveConstants.kRearLeftEncoderOffset);

  private final SwerveModule m_rearRight = //Q3
      new SwerveModule(
          DriveConstants.kRearRightDriveMotorPort,
          DriveConstants.kRearRightTurningMotorPort,
          DriveConstants.kRearRightTurningEncoderPorts,
          DriveConstants.kRearRightDriveReversed,
          DriveConstants.kRearRightTurningMotorReversed,
          DriveConstants.kRearRightEncoderReversed,
          DriveConstants.kRearRightEncoderOffset);

  private final SwerveModule m_frontRight = //Q4
      new SwerveModule(
          DriveConstants.kFrontRightDriveMotorPort,
          DriveConstants.kFrontRightTurningMotorPort,
          DriveConstants.kFrontRightTurningEncoderPorts,
          DriveConstants.kFrontRightDriveReversed,
          DriveConstants.kFrontRightTurningMotorReversed,
          DriveConstants.kFrontRightEncoderReversed,
          DriveConstants.kFrontRightEncoderOffset);

  private final SwerveModule[] m_modules = new SwerveModule[]{
    m_frontLeft,
    m_rearLeft,
    m_rearRight,
    m_frontRight
  };

  private Translation2d m_idealVelocity = new Translation2d(0.0, 0.0);
  private double m_idealAngularVelocity = 0.0;

  // The gyro sensor uses NavX
  private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);
  private Pose2d m_initialPose = new Pose2d();
  private CameraSubsystem m_cameraSystem;

  private SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = Arrays.stream(m_modules)
      .map(module -> module.getPosition())
      .toArray(size -> new SwerveModulePosition[size]);
    return positions;
  }

  // Odometry class for tracking robot pose
  private SwerveDrivePoseEstimator m_odometry =
      new SwerveDrivePoseEstimator(
        DriveConstants.kDriveKinematics,
        getUncorrectedRotation2d(),
        getPositions(),
        m_initialPose,
        DriveConstants.stateStdDeviations,
        DriveConstants.visionStdDeviations);

  private final Field2d m_field = new Field2d();

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem(CameraSubsystem cameraSystem) {
    m_cameraSystem = cameraSystem;
    m_gyro.enableBoardlevelYawReset(true);
    // We have to wait for the gyro to callibrate before we can reset the gyro
    while (m_gyro.isCalibrating()) {Thread.yield();}
    zeroGyro();

    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config = null;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    AutoBuilder.configure(
        this::getPose, // Robot pose supplier
        this::initOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new PPHolonomicDriveController( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            AutoConstants.kTranslationHolonomicPID, // Translation PID constants
            AutoConstants.kRotationHolonomicPID // Rotation PID constants
        ), // Drive base radius in meters. Distance from robot center to furthest module.
        config, // Default path replanning config. See the API for the options here
        () -> {
          return DriverStation.getAlliance().get() == Alliance.Red;
        },
        this // Reference to this subsystem to set requirements
    );
  }

  public void setPIDSlotID(ClosedLoopSlot slotID) {
    for (SwerveModule module : m_modules) {
      module.setPIDSlotID(slotID);
    }
  }

  @Override
  public void periodic() {
    // Update turning encoder offsets if the robot is stationary.
    double aggregateSpeedMetersPerSecond = 0.0;
    for (SwerveModuleState moduleState : getModuleStates()) {
      aggregateSpeedMetersPerSecond += moduleState.speedMetersPerSecond;
    }
    if (aggregateSpeedMetersPerSecond < DriveConstants.kAggregateSpeedThresholdMetersPerSecond) {
      for (SwerveModule module : m_modules) {
        module.updateTurningEncoderOffset();
      }
    }

    // Integrate swerve state into odometry.
    m_odometry.update(getUncorrectedRotation2d(), getPositions());

    // Integrate PhotonVision pose estimates into odometry.
    if (CameraConstants.kEnable) {
      List<Optional<EstimatedRobotPose>> photonRobotPoseList =
        m_cameraSystem.getFieldRelativePoseEstimators();
      for (Optional<EstimatedRobotPose> estimateOpt : photonRobotPoseList) {
        if (estimateOpt.isPresent()) {
          EstimatedRobotPose estimate = estimateOpt.get();
          m_odometry.addVisionMeasurement(estimate.estimatedPose.toPose2d(),
            estimate.timestampSeconds);
        }
      }
    }

    Pose2d pose = getPose();
    m_field.setRobotPose(pose);
    SmartDashboard.putNumber("X", pose.getX());
    SmartDashboard.putNumber("Y", pose.getY());
    SmartDashboard.putData("Field", m_field);
  }

  public Pose2d getPose() {
    return m_odometry.getEstimatedPosition();
  }

  private double getYaw() {
    // Corrections for how we mounted our RoboRIO
    return Math.toRadians(-m_gyro.getYaw());
  }

  private ChassisSpeeds getSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  public void initOdometry(Pose2d initialPose) {
    m_initialPose = initialPose;
    m_odometry.resetPosition(getUncorrectedRotation2d(), getPositions(), initialPose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward), in m/s.
   * @param ySpeed Speed of the robot in the y direction (right), in m/s.
   * @param rot Angular velocity of the robot, in rad/s.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    Translation2d velocityDesired = new Translation2d(xSpeed, ySpeed);
    m_idealVelocity = DriveConstants.kVelocityProfile.calculateTranslation2d(
      velocityDesired, m_idealVelocity, Constants.kDt);

    m_idealAngularVelocity = DriveConstants.kAngularVelocityProfile.calculate(
      rot, m_idealAngularVelocity, Constants.kDt);

    ChassisSpeeds chassisSpeeds = fieldRelative
      ? ChassisSpeeds.fromFieldRelativeSpeeds(
        m_idealVelocity.getX(), m_idealVelocity.getY(), m_idealAngularVelocity, getEstimatedRotation2d())
      : new ChassisSpeeds(m_idealVelocity.getX(), m_idealVelocity.getY(), m_idealAngularVelocity);
    var swerveModuleStates =
      DriveConstants.kDriveKinematics.toSwerveModuleStates(
        ChassisSpeeds.discretize(chassisSpeeds, Constants.kDt));

    setModuleStates(swerveModuleStates);
  }

  public void stop() {
    for (int i = 0; i < m_modules.length; i++) {
      m_modules[i].stop();
    }
  }

  private void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, Constants.kDt);

    SwerveModuleState[] targetStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(targetSpeeds);
    setModuleStates(targetStates);
  }

  private void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, Constants.SwerveModuleConstants.kMaxSpeedMetersPerSecond);

    for (int i = 0; i < m_modules.length; i++) {
      m_modules[i].setDesiredState(desiredStates[i]);
    }
  }

  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = Arrays.stream(m_modules)
      .map(module -> module.getState())
      .toArray(size -> new SwerveModuleState[size]);
    return states;
  }

  public void zeroGyro() {
    m_gyro.reset();
  }

  /* Raw rotation, assuming robot's starting rotation precisely matched that of the ideal initial pose. */
  private Rotation2d getUncorrectedRotation2d() {
    return new Rotation2d(getYaw() + m_initialPose.getRotation().getRadians());
  }

  /* Estimated rotation, which corrects for imprecision of actual initial pose vs ideal initial pose. */
  private Rotation2d getEstimatedRotation2d() {
    return m_odometry.getEstimatedPosition().getRotation();
  }

  /** Return ideal angular velocity of robot in radians/sec. */
  public double getAngularVelocity() {
    return m_idealAngularVelocity;
  }
}
