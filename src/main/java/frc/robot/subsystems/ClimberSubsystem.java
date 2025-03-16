package frc.robot.subsystems;

import java.util.ArrayList;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.utilities.PIDF;
import frc.robot.utilities.SparkUtil;
import frc.robot.utilities.SparkUtil.PIDFSlot;
import frc.robot.utilities.TunablePIDF;

public class ClimberSubsystem extends SubsystemBase {
  private final SparkFlex m_motor;
  private final RelativeEncoder m_motorEncoder;
  private final DutyCycleEncoder m_encoder = new DutyCycleEncoder(
    ClimberConstants.kEncoderChannelAbs,
    ClimberConstants.kAbsEncoderConversionFactor,
    ClimberConstants.kZeroOffset);
  private final TunablePIDF m_motorPIDF = new TunablePIDF("Climber.velocityPIDF", ClimberConstants.kMotorVelocityPIDFSlot.pidf());
  private final SparkClosedLoopController m_controller;
    
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
        ArrayList<PIDFSlot> pidfSlots = new ArrayList<>() {{
        add(new SparkUtil.PIDFSlot(pidf, ClimberConstants.kVelocityPIDFSlot));
      }};
        SparkUtil.Config config = ClimberConstants.kMotorConfig.withPIDFSlots(pidfSlots);
        SparkUtil.configureMotor(m_motor, config);
      }
    }
   m_controller.setReference(transformSpeed(m_encoder.get(), m_idealSpeed), ControlType.kMAXMotionVelocityControl);
  }
}
