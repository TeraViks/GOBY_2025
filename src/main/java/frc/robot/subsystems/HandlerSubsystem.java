package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HandlerConstants;
import frc.robot.utilities.SparkUtil;

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

  private final double m_intakeSpeed = HandlerConstants.kIntakeSpeed;
  private final double m_ejectSpeed = HandlerConstants.kEjectSpeed;

  public HandlerSubsystem(int motorID, SparkUtil.Config motorConfig) {
    m_chooser.setDefaultOption("Empty", State.EMPTY);
    m_chooser.addOption("Preloaded", State.LOADED);
    SmartDashboard.putData(m_chooser);

    m_motor = new SparkMax(motorID, MotorType.kBrushless);
    SparkUtil.configureMotor(m_motor, motorConfig);

    m_encoder = m_motor.getEncoder();

    //TODO: sensor configs here
  }

  public void initializePreloaded() {
    m_state = m_chooser.getSelected();
  }

  public void intake() {
    switch (m_state) {
      case EMPTY:
      case CANCELLING: {
        m_motor.set(m_intakeSpeed);
        m_state = State.INTAKING;
        break;
      }
      case INTAKING:
      case LOADING:
      case LOADED:
      case EJECTING: {
        break;
      }
    }
  }

  public void cancelIntake() {
    switch (m_state) {
      case INTAKING: {
        m_motor.stopMotor();
        m_state = State.CANCELLING;
        break;
      }
      case EMPTY:
      case LOADING:
      case LOADED:
      case EJECTING:
      case CANCELLING: {
        break;
      }
    }
  }

  public void eject() {
    switch (m_state) {
      case EMPTY:
      case INTAKING:
      case LOADING:
      case EJECTING:
      case CANCELLING: {
        break;
      }
      case LOADED: {
        m_motor.set(m_ejectSpeed);
        m_state = State.EJECTING;
        break;
      }
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
