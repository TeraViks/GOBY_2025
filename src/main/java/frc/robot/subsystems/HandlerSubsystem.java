package frc.robot.subsystems;

import java.util.ArrayList;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HandlerConstants;
import frc.robot.utilities.PIDF;
import frc.robot.utilities.SparkUtil;
import frc.robot.utilities.SparkUtil.PIDFSlot;
import frc.robot.utilities.TunablePIDF;

public class HandlerSubsystem extends SubsystemBase {
  private enum State {
    EMPTY,
    INTAKING_CORAL,
    INTAKING_ALGAE,
    LOADING_CORAL,
    LOADED_CORAL,
    LOADED_ALGAE,
    EJECTING_CORAL,
    EJECTING_ALGAE,
    CANCELLING_CORAL,
    CANCELLING_ALGAE
  }

  private static final TunablePIDF motorPIDF = new TunablePIDF("Handler.motorPIDF",
    HandlerConstants.kMotorPIDFVelocity);

  private State m_state = State.EMPTY;
  private final SparkMax m_motor;
  private final SparkClosedLoopController m_motorController;
  private final RelativeEncoder m_encoder;
  private final Pololu4079 m_algaeProxSensor;
  private final Pololu4079 m_backProxSensor;
  private final Pololu4079 m_frontProxSensor;
  private final Pololu4079 m_distanceSensor;
  private final SendableChooser<State> m_chooser = new SendableChooser<>();
  private double m_ejectDelayStartTime = 0.0;

  public HandlerSubsystem(int motorID, SparkUtil.Config motorConfig) {
    m_chooser.setDefaultOption("Empty", State.EMPTY);
    m_chooser.addOption("Preloaded", State.LOADED_CORAL);
    SmartDashboard.putData(m_chooser);

    m_motor = new SparkMax(motorID, MotorType.kBrushless);
    m_motorController = m_motor.getClosedLoopController();
    SparkUtil.configureMotor(m_motor, motorConfig);
    m_encoder = m_motor.getEncoder();

    m_algaeProxSensor = new Pololu4079(
      HandlerConstants.kAlgaeSensorInput, 
      HandlerConstants.kAlgaeSensorProxThreshold, 
      HandlerConstants.kDebounceTime);
    m_backProxSensor = new Pololu4079(
      HandlerConstants.kBackSensorInput, 
      HandlerConstants.kBackSensorProxThreshold, 
      HandlerConstants.kDebounceTime);
    m_frontProxSensor = new Pololu4079(
      HandlerConstants.kFrontSensorInput, 
      HandlerConstants.kFrontSensorProxThreshold, 
      HandlerConstants.kDebounceTime);
    m_distanceSensor = new Pololu4079(
      HandlerConstants.kDistanceSensorInput, 
      HandlerConstants.kDistanceSensorProxThreshold, 
      HandlerConstants.kDebounceTime);
  }

  public void initializePreloaded() {
    m_state = m_chooser.getSelected();
  }

  public double getDistanceSensorMeasurement() {
    return m_distanceSensor.getDistance();
  }

  public void intakeCoral() {
    switch (m_state) {
      case EMPTY:
      case CANCELLING_CORAL: {
        m_motorController.setReference(
          HandlerConstants.kIntakeSpeedCoral, 
          ControlType.kMAXMotionVelocityControl, 
          HandlerConstants.kPIDFSlotVelocity);
        m_state = State.INTAKING_CORAL;
        break;
      }
      case INTAKING_CORAL:
      case INTAKING_ALGAE:
      case LOADING_CORAL:
      case LOADED_CORAL:
      case LOADED_ALGAE:
      case EJECTING_CORAL:
      case EJECTING_ALGAE:
      case CANCELLING_ALGAE: {
        break;
      }
    }
  }

  public void intakeAlgae() {
    switch (m_state) {
      case EMPTY:
      case CANCELLING_ALGAE: {
        m_motorController.setReference(
          HandlerConstants.kIntakeSpeedAlgae, 
          ControlType.kMAXMotionVelocityControl, 
          HandlerConstants.kPIDFSlotVelocity);
        m_state = State.INTAKING_ALGAE;
        break;
      }
      case INTAKING_CORAL:
      case INTAKING_ALGAE:
      case LOADING_CORAL:
      case LOADED_CORAL:
      case LOADED_ALGAE:
      case EJECTING_CORAL:
      case EJECTING_ALGAE:
      case CANCELLING_CORAL: {
        break;
      }
    }
  }

  public void cancelIntake() {
    switch (m_state) {
      case INTAKING_CORAL: {
        m_motor.stopMotor();
        m_state = State.CANCELLING_CORAL;
        break;
      }
      case INTAKING_ALGAE: {
        m_motor.stopMotor();
        m_state = State.CANCELLING_ALGAE;
        break;
      }
      case EMPTY:
      case LOADING_CORAL:
      case LOADED_CORAL:
      case LOADED_ALGAE:
      case EJECTING_CORAL:
      case EJECTING_ALGAE:
      case CANCELLING_CORAL:
      case CANCELLING_ALGAE: {
        break;
      }
    }
  }

  private static double getTimeSeconds() {
    return (double)RobotController.getFPGATime() / 1_000_000.0;
  }

  public void eject() {
    switch (m_state) {
      case EMPTY:
      case INTAKING_CORAL:
      case INTAKING_ALGAE:
      case LOADING_CORAL:
      case EJECTING_CORAL:
      case EJECTING_ALGAE:
      case CANCELLING_CORAL:
      case CANCELLING_ALGAE: {
        break;
      }
      case LOADED_ALGAE: {
        SparkUtil.Config motorConfig = 
          HandlerConstants.kmotorConfig.withCurrentLimit(HandlerConstants.kNormalCurrentLimit);
        SparkUtil.configureMotor(m_motor, motorConfig);
        m_motorController.setReference(
          HandlerConstants.kEjectSpeedAlgae,
          ControlType.kMAXMotionVelocityControl,
          HandlerConstants.kPIDFSlotVelocity);
        m_ejectDelayStartTime = getTimeSeconds();
        m_state = State.EJECTING_ALGAE;
        break;
      }
      case LOADED_CORAL: {
        m_motorController.setReference(
          HandlerConstants.kEjectSpeedCoral,
          ControlType.kMAXMotionVelocityControl,
          HandlerConstants.kPIDFSlotVelocity);
        m_ejectDelayStartTime = getTimeSeconds();
        m_state = State.EJECTING_CORAL;
        break;
      }
    }
  }

  public boolean isLoadedCoral() {
    return m_state == State.LOADED_CORAL;
  }

  public boolean isLoadedAlgae() {
    return m_state == State.LOADED_ALGAE;
  }

  private void updateConstants() {
    if (motorPIDF.hasChanged()) {
      PIDF pidf = motorPIDF.get();
      ArrayList<PIDFSlot> pidfSlots = new ArrayList<>() {{
        add(new SparkUtil.PIDFSlot(pidf, HandlerConstants.kPIDFSlotVelocity));
      }};
      SparkUtil.Config motorConfig = HandlerConstants.kmotorConfig.withPIDFSlots(pidfSlots);
      SparkUtil.configureMotor(m_motor, motorConfig);
    }
  }

  @Override
  public void periodic() {
    updateConstants();
    SmartDashboard.putString("Handler State: ", m_state.name());
    switch (m_state) {
      case EMPTY: {
        break;
      }
      case INTAKING_CORAL: {
        m_motorController.setReference(
          HandlerConstants.kIntakeSpeedCoral,
          ControlType.kMAXMotionVelocityControl,
          HandlerConstants.kPIDFSlotVelocity);
        if (m_frontProxSensor.isProximate()) {
          m_state = State.LOADING_CORAL;
        }
        break;
      }
      case INTAKING_ALGAE: {
        m_motorController.setReference(
          HandlerConstants.kIntakeSpeedAlgae, 
          ControlType.kMAXMotionVelocityControl,
          HandlerConstants.kPIDFSlotVelocity);
        if (m_algaeProxSensor.isProximate()) {
          
          m_motorController.setReference(
            m_encoder.getPosition(),
            ControlType.kMAXMotionPositionControl,
            HandlerConstants.kPIDFSlotPosition);
          SparkUtil.Config motorConfig = 
            HandlerConstants.kmotorConfig.withCurrentLimit(HandlerConstants.kHoldingCurrentLimit);
          SparkUtil.configureMotor(m_motor, motorConfig);
          m_state = State.LOADED_ALGAE;
        }
      }
      case LOADING_CORAL: {
        if (m_backProxSensor.isProximate()){
          m_motor.stopMotor();
          m_state = State.LOADED_CORAL;
        }
        break;
      }
      case LOADED_CORAL: {
        break;
      }
      case LOADED_ALGAE: {
        break;
      }
      case EJECTING_CORAL: {
        double currentTime = getTimeSeconds();
        if (currentTime - m_ejectDelayStartTime >= HandlerConstants.kEjectDelaySeconds) {
          m_motor.stopMotor();
          m_state = State.EMPTY;
        }
        break;
      }
      case EJECTING_ALGAE: {
        double currentTime = getTimeSeconds();
        if (currentTime - m_ejectDelayStartTime >= HandlerConstants.kEjectDelaySeconds) {
          m_motor.stopMotor();
          m_state = State.EMPTY;
        }
        break;
      }
      case CANCELLING_CORAL: {
        if (m_frontProxSensor.isProximate()) {
          m_motorController.setReference(
            HandlerConstants.kIntakeSpeedCoral,
            ControlType.kMAXMotionVelocityControl,
            HandlerConstants.kPIDFSlotVelocity);
          m_state = State.LOADING_CORAL;
        } else {
          m_state = State.EMPTY;
        }
        break;
      }
      case CANCELLING_ALGAE: {
        if (m_algaeProxSensor.isProximate()) {
          m_state = State.LOADED_ALGAE;
        } else {
          m_state = State.EMPTY;
        }
        break;
      }
    }
  }
}
