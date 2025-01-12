package frc.robot.utilities;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;

public class TrapezoidalConstraint {
  private final double m_maxSpeed;
  private final double m_maxAcceleration;
  private final double m_maxDeceleration;

  public TrapezoidalConstraint(double maxSpeed, double maxAcceleration, double maxDeceleration) {
    assert(maxSpeed >= 0.0);
    assert(maxAcceleration >= 0.0);
    assert(maxDeceleration >= 0.0); // We negate this during calculation.
    m_maxSpeed = maxSpeed;
    m_maxAcceleration = maxAcceleration;
    m_maxDeceleration = maxDeceleration;
  }

  public double calculate(double desiredVelocity, double currentVelocity, double deltaT) {
    double desiredAcceleration = (desiredVelocity - currentVelocity) / deltaT;
    double maxDeceleration, maxAcceleration;
    if (currentVelocity >= 0.0) {
      maxDeceleration = -m_maxDeceleration;
      maxAcceleration = m_maxAcceleration;
    } else {
      maxDeceleration = -m_maxAcceleration;
      maxAcceleration = m_maxDeceleration;
    }
    double clampedAcceleration = MathUtil.clamp(desiredAcceleration, maxDeceleration, maxAcceleration);
    double deltaV = clampedAcceleration * deltaT;
    double unclampedNewVelocity = currentVelocity + deltaV;
    double clampedNewVelocity = MathUtil.clamp(unclampedNewVelocity, -m_maxSpeed, m_maxSpeed);
    return clampedNewVelocity;
  }

  public Translation2d calculateTranslation2d(Translation2d desiredVelocity, Translation2d currentVelocity, double deltaT) {
    if (desiredVelocity.equals(currentVelocity)) {
      return desiredVelocity;
    }
    double desiredVelocityMagnitude = desiredVelocity.getNorm();
    if (desiredVelocityMagnitude != 0.0) {
      double clampedVelocityMagnitude = MathUtil.clamp(desiredVelocityMagnitude, -m_maxSpeed, m_maxSpeed);
      double velocityScalar = clampedVelocityMagnitude / desiredVelocityMagnitude;
      desiredVelocity = desiredVelocity.times(velocityScalar);
    }
    Translation2d desiredDeltaVelocity = desiredVelocity.minus(currentVelocity);
    Translation2d desiredAcceleration = desiredDeltaVelocity.div(deltaT);
    double desiredAccelerationMagnitude = desiredAcceleration.getNorm();
    if (desiredAccelerationMagnitude == 0.0) {
      return currentVelocity;
    }
    double clampedAccelerationMagnitude = MathUtil.clamp(desiredAccelerationMagnitude, -m_maxDeceleration, m_maxAcceleration);
    double accelerationScalar = clampedAccelerationMagnitude / desiredAccelerationMagnitude;
    Translation2d newVelocity = currentVelocity.plus(desiredDeltaVelocity.times(accelerationScalar));
    return newVelocity;
  }
}