package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.utilities.PIDF;
import frc.robot.utilities.SparkUtil;
import frc.robot.utilities.TunablePIDF;

public class ClimberSubsystem extends SubsystemBase {
  private final SparkFlex m_leftMotor;
  private final SparkFlex m_rightMotor;
  private final RelativeEncoder m_encoder;
  private final TunablePIDF m_motorPIDF =
    new TunablePIDF("Climber.velocityPIDF", ClimberConstants.kVelocityPIDF);
  private final PIDController m_PIDController = new PIDController(
    m_motorPIDF.get().p(),
    m_motorPIDF.get().i(),
    m_motorPIDF.get().d(),
    m_motorPIDF.get().ff());

  private double m_idealSpeed;

  public ClimberSubsystem() {
    m_leftMotor = new SparkFlex(ClimberConstants.kLeftMotorID, MotorType.kBrushless);
    SparkUtil.configureMotor(m_leftMotor, ClimberConstants.kMotorConfig);

    m_rightMotor = new SparkFlex(ClimberConstants.kRightMotorID, MotorType.kBrushless);
    SparkUtil.configureFollowerMotor(
      m_rightMotor,
      ClimberConstants.kMotorConfig.withInvert(!ClimberConstants.kInvertLeftMotor),
      m_leftMotor
    );

    m_encoder = m_leftMotor.getEncoder();
    m_encoder.setPosition(ClimberConstants.kInitialAngle);
  }

  public void stopClimber() {
    m_leftMotor.stopMotor();
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
      SmartDashboard.putNumber("Climber.velocity", m_encoder.getVelocity());
      if (m_motorPIDF.hasChanged()) {
        PIDF pidf = m_motorPIDF.get();
        m_PIDController.setPID(pidf.p(), pidf.i(), pidf.d());
      }
    }
    m_leftMotor.set(m_PIDController.calculate(
      m_encoder.getVelocity(),
      transformSpeed(m_encoder.getPosition(), m_idealSpeed)
    ));
  }
}