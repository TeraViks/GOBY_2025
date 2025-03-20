package frc.robot.subsystems;

import java.util.ArrayList;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.utilities.PIDF;
import frc.robot.utilities.SparkUtil;
import frc.robot.utilities.SparkUtil.PIDFSlot;
import frc.robot.utilities.TunablePIDF;

public class ClimberSubsystem extends SubsystemBase {
  private final SparkFlex m_leftMotor;
  private final SparkFlex m_rightMotor;
  private final SparkClosedLoopController m_controller;
  private final RelativeEncoder m_encoder;
  private final TunablePIDF m_motorPIDF =
    new TunablePIDF("Climber.velocityPIDF", ClimberConstants.kVelocityPIDF);

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

    m_controller = m_leftMotor.getClosedLoopController();

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
    // if (position >= ClimberConstants.kMaxRange - ClimberConstants.kAngleTolerance &&
    //     speed > 0.0) {
    //   return speed * (ClimberConstants.kMaxRange - position);
    // } else if (
    //     position <= ClimberConstants.kMinRange + ClimberConstants.kAngleTolerance &&
    //     speed < 0.0) {
    //   return speed * (position - ClimberConstants.kMinRange);
    // } else {
    //   return speed;
    // }
    if (position < ClimberConstants.kMinRange && speed < 0.0) {
      return 0.0;
    } else if (position > ClimberConstants.kMaxRange && speed > 0.0) {
      return 0.0;
    } else {
      return speed;
    }
  }

  private void updateConstants() {
    SmartDashboard.putNumber("Climber.velocity", m_encoder.getVelocity());
    if (m_motorPIDF.hasChanged()) {
      PIDF pidf = m_motorPIDF.get();
      ArrayList<PIDFSlot> pidfSlots = new ArrayList<>() {{
        add(new SparkUtil.PIDFSlot(pidf, ClimberConstants.kVelocitySlot));
      }};
      SparkUtil.Config leftMotorConfig = ClimberConstants.kMotorConfig.withPIDFSlots(pidfSlots);
      SparkUtil.configureMotor(m_leftMotor, leftMotorConfig);
      SparkUtil.Config rightMotorConfig =
        leftMotorConfig.withInvert(!ClimberConstants.kInvertLeftMotor);
      SparkUtil.configureFollowerMotor(m_rightMotor, rightMotorConfig, m_leftMotor);
    }
  }

  @Override
  public void periodic() {
    updateConstants();
    m_controller.setReference(transformSpeed(m_encoder.getPosition(), m_idealSpeed),
     ControlType.kMAXMotionVelocityControl);
  }
}