// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.io.UncheckedIOException;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.utilities.TunablePIDF;
import frc.robot.utilities.ValueCache;

public class SwerveModule {
  public static final TunablePIDF tunableTeleopTurningPIDF =
    new TunablePIDF("TeleopTurning", SwerveModuleConstants.kTeleopTurningPIDF);

  private final SparkMax m_driveMotor;
  private final SparkMax m_turningMotor;

  private ClosedLoopSlot m_PIDFSlotID = SwerveModuleConstants.kDefaultPIDFSlotID;

  private final SparkClosedLoopController m_drivePIDController;
  private final SparkClosedLoopController m_turningPIDController;

  private final RelativeEncoder m_driveEncoder;
  private final CANcoder m_absoluteRotationEncoder;
  private final RelativeEncoder m_turningEncoder;

  private final Rotation2d m_absoluteRotationEncoderOffset;
  private Rotation2d m_turningEncoderOffset;

  private final ValueCache<Double> m_drivePositionCache;
  private final ValueCache<Double> m_driveVelocityCache;
  private final ValueCache<Angle> m_absoluteAngleCache;
  private final ValueCache<Double> m_turningCache;
  private Rotation2d m_prevAngle;
  private double m_lastViableDrivePosition = 0.0;
  private double m_lastViableDriveVelocity = 0.0;
  private double m_lastViableTurningPosition = 0.0;

  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int turningEncoderChannel,
      boolean driveMotorReversed,
      boolean turningMotorReversed,
      boolean turningEncoderReversed,
      Rotation2d encoderOffset) {
    m_driveMotor = new SparkMax(driveMotorChannel, MotorType.kBrushless);
    
    SparkMaxConfig config = new SparkMaxConfig();
    config.closedLoopRampRate(SwerveModuleConstants.kDriveMotorRampRate)
      .inverted(driveMotorReversed)
      .smartCurrentLimit(SwerveModuleConstants.kDriveMotorCurrentLimit)
      .idleMode(IdleMode.kBrake);
    config.encoder
      .positionConversionFactor(SwerveModuleConstants.kDrivePositionConversionFactor)
      .velocityConversionFactor(SwerveModuleConstants.kDriveVelocityConversionFactor);
    config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    SwerveModuleConstants.kAutoDrivePIDF.controllerSet(config.closedLoop, SwerveModuleConstants.kAutoPIDFSlotID);
    SwerveModuleConstants.kTeleopDrivePIDF.controllerSet(config.closedLoop, SwerveModuleConstants.kTeleopPIDFSlotID);

    REVLibError configureError = m_driveMotor.configure(config, ResetMode.kResetSafeParameters, Constants.kPersistMode);
    if (configureError != REVLibError.kOk) {
      throw new UncheckedIOException("Failed to configure drive motor", new IOException());
    }

    m_driveEncoder = m_driveMotor.getEncoder();
    m_drivePIDController = m_driveMotor.getClosedLoopController();

    m_drivePositionCache = new ValueCache<Double>(this::getPlausibleDrivePosition, SwerveModuleConstants.kValueCacheTtlMicroseconds);
    m_driveVelocityCache = new ValueCache<Double>(this::getPlausibleDriveVelocity, SwerveModuleConstants.kValueCacheTtlMicroseconds);
 
    m_absoluteRotationEncoderOffset = encoderOffset;

    m_absoluteRotationEncoder = new CANcoder(turningEncoderChannel);
    var turningEncoderConfigurator = m_absoluteRotationEncoder.getConfigurator();
    var encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.SensorDirection = turningEncoderReversed
      ? SensorDirectionValue.Clockwise_Positive
      : SensorDirectionValue.CounterClockwise_Positive;
    encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = SwerveModuleConstants.kAbsoluteSensorDiscontinuityPoint;
    turningEncoderConfigurator.apply(encoderConfig);

    m_absoluteRotationEncoder.getAbsolutePosition().getValue();

    m_absoluteAngleCache =
      new ValueCache<Angle>(() -> {
        return m_absoluteRotationEncoder.getAbsolutePosition().getValue();
      }, SwerveModuleConstants.kValueCacheTtlMicroseconds);

    m_turningMotor = new SparkMax(turningMotorChannel, MotorType.kBrushless);

    SparkMaxConfig turningConfig = new SparkMaxConfig();
    turningConfig.closedLoopRampRate(SwerveModuleConstants.kTurningMotorRampRate)
      .inverted(turningMotorReversed)
      .smartCurrentLimit(SwerveModuleConstants.kTurningMotorCurrentLimit)
      .idleMode(IdleMode.kBrake);
    turningConfig.encoder
      .positionConversionFactor(SwerveModuleConstants.kTurningPositionConversionFactor)
      .velocityConversionFactor(SwerveModuleConstants.kTurningVelocityConversionFactor);
    turningConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    m_turningPIDController = m_turningMotor.getClosedLoopController();
    SwerveModuleConstants.kAutoTurningPIDF.controllerSet(turningConfig.closedLoop, SwerveModuleConstants.kAutoPIDFSlotID);
    SwerveModuleConstants.kTeleopTurningPIDF.controllerSet(turningConfig.closedLoop, SwerveModuleConstants.kTeleopPIDFSlotID);
    
    REVLibError configureTurningMotorError = m_turningMotor.configure(turningConfig, ResetMode.kResetSafeParameters, Constants.kPersistMode);
    if (configureTurningMotorError != REVLibError.kOk) {
      throw new UncheckedIOException("Failed to configure turning motor", new IOException());
    }

    m_turningEncoder = m_turningMotor.getEncoder();

    // Stabilize encoder readings before initializing the cache.
    m_turningEncoder.setPosition(0.0);
    while (
      Math.abs(m_turningEncoder.getPosition())
      > SwerveModuleConstants.kTurningEncoderStabilizeToleranceRadians
    ) {Thread.yield();}
    m_turningCache = new ValueCache<Double>(this::getPlausibleTurningPosition,
      SwerveModuleConstants.kValueCacheTtlMicroseconds);
    updateTurningEncoderOffset();
    m_prevAngle = getUnconstrainedRotation2d();
  }

  private double getPlausibleDrivePosition() {
    double position = m_driveEncoder.getPosition();
    if (m_driveMotor.getLastError() == REVLibError.kOk) {
      m_lastViableDrivePosition = position;
    }
    return m_lastViableDrivePosition;
  }

  private double getPlausibleDriveVelocity() {
    double velocity = m_driveEncoder.getVelocity();
    if (m_driveMotor.getLastError() == REVLibError.kOk) {
      m_lastViableDriveVelocity = velocity;
    }
    return m_lastViableDriveVelocity;
  }

  private double getPlausibleTurningPosition() {
    double position = m_turningEncoder.getPosition();
    if (m_turningMotor.getLastError() == REVLibError.kOk) {
      m_lastViableTurningPosition = position;
    }
    return m_lastViableTurningPosition;
  }

  public void setPIDSlotID(ClosedLoopSlot slotID) {
    m_PIDFSlotID = slotID;
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
      m_driveVelocityCache.get(),
      getRotation2d()
    );
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
      m_drivePositionCache.get(),
      getRotation2d()
    );
  }

  public void stop() {
    m_driveMotor.stopMotor();
    m_turningMotor.stopMotor();
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees. Note that providing
    // m_prevAngle as the current angle is not as truthful as providing getRotation2d(), because the
    // module may not have yet reached the angle it was most recently commanded to. However, there
    // is inherent instability in the algorithm if we tell the truth using getRotation2d(), because
    // commanding a position and reading current position are both asynchronous.
    desiredState.optimize(m_prevAngle);

    m_drivePIDController.setReference(desiredState.speedMetersPerSecond, ControlType.kVelocity, m_PIDFSlotID);
    
    if (!desiredState.angle.equals(m_prevAngle)) {
      // deltaAngle is in [-pi..pi], which is added (intentionally unconstrained) to m_prevAngle.
      // This causes the module to turn e.g. 4 degrees rather than -356 degrees.
      Rotation2d deltaAngle = desiredState.angle.minus(m_prevAngle);
      // Avoid Rotation2d.plus() here, since it constrains the result to [-pi..pi].
      Rotation2d angle = Rotation2d.fromRadians(m_prevAngle.getRadians() + deltaAngle.getRadians());
      // Take care to cancel out the encoder offset when setting the position.
      m_turningPIDController.setReference(angle.getRadians() + m_turningEncoderOffset.getRadians(),
        ControlType.kPosition, m_PIDFSlotID);
      m_prevAngle = angle;
    }
  }

  private Rotation2d getAbsoluteRotation2d() {
    return new Rotation2d(m_absoluteAngleCache.get()).minus(m_absoluteRotationEncoderOffset);
  }

  /**
   * Update relative turning encoder offset to correspond to the absolute turning encoder. This
   * should only be done when the robot is (at least nearly) stationary. Unfortunately, the
   * relative encoder in the NEO isn't very accurate, and angle mismatches up to ~15 degrees
   * commonly occur in the absence of such updates.
   */
  public void updateTurningEncoderOffset() {
    Rotation2d absolute = getAbsoluteRotation2d();
    double relativeRadians = m_turningCache.get();
    m_turningEncoderOffset = Rotation2d.fromRadians(relativeRadians).minus(absolute);
  }

  private Rotation2d getRotation2d() {
    return Rotation2d.fromRadians(m_turningCache.get()).minus(m_turningEncoderOffset);
  }

  private Rotation2d getUnconstrainedRotation2d() {
    // Avoid Rotation2d.minus() here, since it constrains the result to [-pi..pi].
    return Rotation2d.fromRadians(m_turningCache.get() - m_turningEncoderOffset.getRadians());
  }
}
