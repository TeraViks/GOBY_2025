package frc.robot.subsystems;

import java.io.IOException;
import java.io.UncheckedIOException;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.HandlerConstants;

public class HandlerSubsystem extends SubsystemBase {
  private enum State {
    EMPTY,
    INTAKING,
    LOADING,
    LOADED,
    EJECTING,
    CANCELLING
  }

  private State m_state = State.EMPTY;
  private final SparkMax m_motor;
  private final RelativeEncoder m_encoder;
  private final SendableChooser<State> m_chooser = new SendableChooser<>();

  private double m_intakeSpeed = HandlerConstants.kIntakeSpeed;
  private double m_ejectSpeed = HandlerConstants.kEjectSpeed;

  public HandlerSubsystem(int driveMotorChannel) {
    m_chooser.setDefaultOption("Empty", State.EMPTY);
    m_chooser.addOption("Preloaded", State.LOADED);
    SmartDashboard.putData(m_chooser);

    m_motor = new SparkMax(driveMotorChannel, MotorType.kBrushless);

    SparkMaxConfig config = new SparkMaxConfig();
    config.inverted(HandlerConstants.kMotorReversed)
      .smartCurrentLimit(HandlerConstants.kMotorCurrentLimit)
      .idleMode(IdleMode.kBrake);
    config.encoder
      .velocityConversionFactor(HandlerConstants.kVelocityConversionFactor);
    config.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    REVLibError configureError = m_motor.configure(config, ResetMode.kResetSafeParameters, Constants.kPersistMode);
    if (configureError != REVLibError.kOk) {
      throw new UncheckedIOException("Failed to configure handler motor", new IOException());
    }

    m_encoder = m_motor.getEncoder();

    //TODO: sensor configs here
  }

  public void initializePreloaded() {
    m_state = m_chooser.getSelected();
  }

  public void intake() {
    if (m_state != State.LOADED) {
      m_state = State.INTAKING;
      m_motor.set(m_intakeSpeed);
    }
  }

  public void cancelIntake() {
    if (m_state != State.LOADING && m_state != State.LOADED) {
      m_motor.stopMotor();
      m_state = State.CANCELLING;
    }
  }

  public void eject() {
    if (m_state == State.LOADED) {
      m_state = State.EJECTING;
      m_motor.set(m_ejectSpeed);
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("Handler State: ", m_state.name());
    switch (m_state) {
      case EMPTY: {
        break;
      }
      case INTAKING: {
        m_motor.set(m_intakeSpeed);
        // if (first sensor sees) {
        //   m_state = State.LOADING;
        // }
        break;
      }
      case LOADING: {
        // if (base sensor sees){
        //   m_motor.stopMotor();
        //   m_state = State.LOADED;
        // }
        break;
      }
      case LOADED: {
        break;
      }
      case EJECTING: {
        // if (first sensor doesn't see) {
        //   m_motor.stopMotor();
        //   m_state = State.EMPTY;
        // }
        break;
      }
      case CANCELLING: {
        // if (first sensor sees) {
        //   intake();
        // } else {
        //   m_state = State.EMPTY;
        // }
        break;
      }
    }
  }
}
