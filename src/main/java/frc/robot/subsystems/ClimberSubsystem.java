package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.utilities.PIDF;
import frc.robot.utilities.SparkUtil;
import frc.robot.utilities.TunablePIDF;

public class ClimberSubsystem extends SubsystemBase {
  private final SparkFlex m_motor;
  private final Encoder m_encoder = new Encoder(
    ClimberConstants.kEncoderChannelA,
    ClimberConstants.kEncoderChannelB,
    false,
    EncodingType.k4X);
  private final TunablePIDF m_motorPIDF = new TunablePIDF("Climber.velocityPIDF", ClimberConstants.kMotorPIDF);
  private final PIDController m_PIDController = new PIDController(m_motorPIDF.get().p(), m_motorPIDF.get().i(), m_motorPIDF.get().d(), m_motorPIDF.get().ff());
  private double m_idealSpeed;

  public ClimberSubsystem() {
    m_motor = new SparkFlex(ClimberConstants.kMotorID, MotorType.kBrushless);
    SparkUtil.configureMotor(m_motor, ClimberConstants.kMotorConfig);
    m_encoder.setDistancePerPulse(ClimberConstants.kAngleConversionFactor);

    initialize();
  }

  public void initialize() {
    m_encoder.getRaw();
  }

  public void recalibrate() {
    setSpeed(ClimberConstants.kRecalibratingSpeed);
  }

  public void stopClimber() {
    m_motor.stopMotor();
  }

  public void setSpeed(double speed) {
    m_idealSpeed = speed;
  }

  @Override
  public void periodic() {
    if (m_motorPIDF.hasChanged()) {
      PIDF pidf = m_motorPIDF.get();
      m_PIDController.setPID(pidf.p(), pidf.i(), pidf.d());
    }
    m_motor.set(m_PIDController.calculate(m_encoder.getRaw(), m_idealSpeed));
  }
}
