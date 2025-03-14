package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.utilities.PIDF;
import frc.robot.utilities.SparkUtil;
import frc.robot.utilities.TunablePIDF;

public class ClimberSubsystem extends SubsystemBase {
  private final SparkFlex m_motor;
  private final RelativeEncoder m_motorEncoder;
  private final DutyCycleEncoder m_encoder = new DutyCycleEncoder(
    ClimberConstants.kEncoderChannelAbs,
    ClimberConstants.kMaxRange,
    ClimberConstants.kZeroOffset);
  private final TunablePIDF m_motorPIDF = new TunablePIDF("Climber.velocityPIDF", ClimberConstants.kMotorPIDF);
  private final PIDController m_PIDController = new PIDController(
    m_motorPIDF.get().p(), 
    m_motorPIDF.get().i(), 
    m_motorPIDF.get().d(), 
    m_motorPIDF.get().ff());
    
  private double m_idealSpeed;

  public ClimberSubsystem() {
    m_motor = new SparkFlex(ClimberConstants.kMotorID, MotorType.kBrushless);
    m_motorEncoder = m_motor.getEncoder();
    SparkUtil.configureMotor(m_motor, ClimberConstants.kMotorConfig);
  }

  public void stopClimber() {
    m_motor.stopMotor();
  }

  public void setSpeed(double speed) {
    m_idealSpeed = speed;
  }

  private double transformSpeed(double position, double speed) {
    if (position >= ClimberConstants.kMaxRange - ClimberConstants.kAngleTolerance &&
        speed > 0.0) {
      return speed * ((ClimberConstants.kMaxRange - position) / 
        ClimberConstants.kAngleTolerance);
    } else if (
        position <= ClimberConstants.kMinRange + ClimberConstants.kAngleTolerance &&
        speed < 0.0) {
      return speed * ((position - ClimberConstants.kMinRange) /
        ClimberConstants.kAngleTolerance);
    } else {
      return speed;
    }
  }

  @Override
  public void periodic() {
    if (Constants.kEnableTuning) {
      SmartDashboard.putNumber("Climber.angle", m_encoder.get());
      SmartDashboard.putNumber("Climber.velocity", m_motorEncoder.getVelocity());
      if (m_motorPIDF.hasChanged()) {
        PIDF pidf = m_motorPIDF.get();
        m_PIDController.setPID(pidf.p(), pidf.i(), pidf.d());
      }
    }
    m_motor.set(m_PIDController.calculate(m_motorEncoder.getVelocity(), transformSpeed(m_encoder.get(), m_idealSpeed)));
  }
}
