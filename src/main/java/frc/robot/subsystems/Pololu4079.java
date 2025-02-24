package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utilities.Time;
import frc.robot.utilities.ValueCache;

/** Distance/proximity sensor based on the Pololu 4079 (https://www.pololu.com/product/4079). */
public class Pololu4079 extends SubsystemBase {
  private final DutyCycle m_dutyCycle;
  private final double m_max_distance; // Meters.
  // Debounce time is rounded up to a multiple of the scheduling period.
  private final double m_debounceSec;
  private final ValueCache<Double> m_distanceCache; // Meters.
  // Refresh distance reading at most once per scheduling period.
  private static final long ttlMicroseconds = (long)(Constants.kDt * 1_000_000.0);
  // Time at which detection transitioned to proximate, or infinity if not proximate. Only used
  // if debouncing is enabled.
  private double m_proximateT0Sec;

  private Pololu4079(DigitalInput digitalInput, double max_distance, double debounceSec) {
    m_dutyCycle = new DutyCycle(digitalInput);
    m_max_distance = max_distance;
    m_debounceSec = debounceSec;
    m_distanceCache = new ValueCache<>(this::getDistanceImpl, ttlMicroseconds);
  }

  /**
   * Pololu4079(channel) constructs a distance sensor. isProximate() always returns false
   * when so constructed.
   */
  public Pololu4079(int channel) {
    this(new DigitalInput(channel), -1.0, 0.0);
  }

  /**
   * Pololu4079(channel, max_distance, debounceSec) initializes a proximity sensor such that
   * isProximate() considers detections no more than max_distance meters away as proximate, as
   * long as proximate detection has persisted for at least debounceSec seconds.
   */
  public Pololu4079(int channel, double max_distance, double debounceSec) {
    this(new DigitalInput(channel), max_distance,
      Math.ceil(debounceSec / Constants.kDt) * Constants.kDt);
  }

  private double getDistanceImpl() {
    // Valid pulse width is approximately 1.0ms to 1.75ms, with ~5% timing uncertainty.
    // A ~2.0ms pulse width indicates no detection, so treat anything above 1.85ms as
    // no detection.
    double distance; // Meters.
    double pulseMs = m_dutyCycle.getHighTimeNanoseconds() / 1_000_000.0;
    if (pulseMs <= 1.85) {
      distance = Math.max(4.0 * (pulseMs - 1.0), 0.0);
    } else {
      distance = Double.POSITIVE_INFINITY;
    }
    return distance;
  }

  public double getDistance() {
    return m_distanceCache.get();
  }

  private boolean isProximateImpl() {
    return getDistance() <= m_max_distance;
  }

  public boolean isProximate() {
    if (m_debounceSec != 0.0) {
      return m_proximateT0Sec + m_debounceSec <= Time.getTimeSeconds();
    } else {
      return isProximateImpl();
    }
  }

  @Override
  public void periodic() {
    // Only read the distance if necessary for debouncing.
    if (m_debounceSec != 0.0) {
      if (isProximateImpl()) {
        if (m_proximateT0Sec == Double.POSITIVE_INFINITY) {
          m_proximateT0Sec = Time.getTimeSeconds();
        }
      } else {
        m_proximateT0Sec = Double.POSITIVE_INFINITY;
      }
    }
  }
}