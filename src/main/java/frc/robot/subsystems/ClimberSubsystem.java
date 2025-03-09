package frc.robot.subsystems;

import com.revrobotics.REVLibError;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.utilities.SparkUtil;

public class ClimberSubsystem extends SubsystemBase {
  private final SparkFlex m_motor;
  private final AbsoluteEncoder m_encoder;
  private final SparkClosedLoopController m_PIDController;
  private double m_idealSpeed;

  private enum State {
    RECALIBRATING,
    OPERATING
  }

  private State m_state;

  public ClimberSubsystem() {
    m_motor = new SparkFlex(ClimberConstants.kMotorID, MotorType.kBrushless);
    SparkUtil.configureMotor(m_motor, ClimberConstants.kMotorConfig);

    m_encoder = m_motor.getAbsoluteEncoder();
    m_PIDController = m_motor.getClosedLoopController();

    initialize();
  }

  public void initialize() {
    m_encoder.getPosition();
    m_state = State.OPERATING;
  }

  public void recalibrate() {
    m_PIDController.setReference(ClimberConstants.kRecalibratingSpeed, ControlType.kVelocity);
    m_state = State.RECALIBRATING;
  }

  public void stopClimber() {
    m_motor.stopMotor();
  }

  public void setSpeed(double speed) {
    m_idealSpeed = speed;
  }

  private double transformSpeed(double position, double speed) {
    if (position >= ClimberConstants.kMaxExtendedAngle - ClimberConstants.kConstrainedRange &&
        speed > 0.0) {
      return speed * ((ClimberConstants.kMaxExtendedAngle - position) / 
        ClimberConstants.kConstrainedRange);
    } else if (
        position <= ClimberConstants.kMinRetractedAngle + ClimberConstants.kConstrainedRange &&
        speed < 0.0) {
      return speed * ((position - ClimberConstants.kMinRetractedAngle) /
        ClimberConstants.kConstrainedRange);
    } else {
      return speed;
    }
  }

  @Override
  public void periodic() {
    switch (m_state) {
      case RECALIBRATING: {
        if (m_encoder.getPosition() <= (ClimberConstants.kHardMin + ClimberConstants.kConstrainedRange) && 
            m_encoder.getPosition() >= (ClimberConstants.kHardMin - ClimberConstants.kConstrainedRange)) {
          initialize();
        }
        break;
      }
      case OPERATING: {
        double position = m_encoder.getPosition();
        double speed = (m_motor.getLastError() == REVLibError.kOk) ? m_idealSpeed : 0.0;
        m_PIDController.setReference(transformSpeed(position, speed), ControlType.kVelocity);
        break;
      }
    }
  }
}
