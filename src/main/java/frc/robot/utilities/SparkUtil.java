package frc.robot.utilities;

import java.io.IOException;
import java.io.UncheckedIOException;
import java.util.ArrayList;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;

public class SparkUtil {
  public record PIDFSlot (PIDF pidf, ClosedLoopSlot slot) {}
  public record Config (
    int currentLimit, double rampRate, boolean invert,
    double velocityConversionFactor, double positionConversionFactor,
    double maxVelocity, double maxAcceleration,
    ArrayList<PIDFSlot> pidfSlots
  ) {
    public Config withInvert(boolean invert) {
      return new Config(
        this.currentLimit, this.rampRate, invert,
        this.velocityConversionFactor, this.positionConversionFactor,
        this.maxVelocity, this.maxAcceleration,
        this.pidfSlots);
    }
  }

  private static void configureMotorImpl(SparkMax motor, Config config, SparkMax leader) {
    SparkMaxConfig motorConfig = new SparkMaxConfig();
    motorConfig
      .smartCurrentLimit(config.currentLimit)
      .closedLoopRampRate(config.rampRate)
      .idleMode(IdleMode.kBrake);
    if (leader != null) {
      motorConfig.follow(leader, config.invert);
    } else {
      motorConfig.inverted(config.invert);
    }
    motorConfig.encoder
      .velocityConversionFactor(config.velocityConversionFactor)
      .positionConversionFactor(config.positionConversionFactor);
    motorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .maxMotion
        .maxVelocity(config.maxVelocity)
        .maxAcceleration(config.maxAcceleration);

    for (PIDFSlot pidfSlot : config.pidfSlots) {
      pidfSlot.pidf.controllerSet(motorConfig.closedLoop, pidfSlot.slot);
    }

    REVLibError configureError = motor.configure(motorConfig, ResetMode.kResetSafeParameters,
      Constants.kPersistMode);
    if (configureError != REVLibError.kOk) {
      throw new UncheckedIOException("Failed to configure motor", new IOException());
    }
  }

  public static void configureMotor(SparkMax motor, Config config) {
    configureMotorImpl(motor, config, null);
  }

  public static void configureFollowerMotor(SparkMax motor, Config config, SparkMax leader) {
    configureMotorImpl(motor, config, leader);
  }

  private static void configureMotorImpl(SparkFlex motor, Config config, SparkFlex leader) {
    SparkFlexConfig motorConfig = new SparkFlexConfig();
    motorConfig
      .smartCurrentLimit(config.currentLimit)
      .closedLoopRampRate(config.rampRate)
      .idleMode(IdleMode.kBrake);
    if (leader != null) {
      motorConfig.follow(leader, config.invert);
    } else {
      motorConfig.inverted(config.invert);
    }
    motorConfig.encoder
      .velocityConversionFactor(config.velocityConversionFactor)
      .positionConversionFactor(config.positionConversionFactor);
    motorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .maxMotion
        .maxVelocity(config.maxVelocity)
        .maxAcceleration(config.maxAcceleration);

    for (PIDFSlot pidfSlot : config.pidfSlots) {
      pidfSlot.pidf.controllerSet(motorConfig.closedLoop, pidfSlot.slot);
    }

    REVLibError configureError = motor.configure(motorConfig, ResetMode.kResetSafeParameters,
      Constants.kPersistMode);
    if (configureError != REVLibError.kOk) {
      throw new UncheckedIOException("Failed to configure motor", new IOException());
    }
  }

  public static void configureMotor(SparkFlex motor, Config config) {
    configureMotorImpl(motor, config, null);
  }

  public static void configureFollowerMotor(SparkFlex motor, Config config, SparkFlex leader) {
    configureMotorImpl(motor, config, leader);
  }
}