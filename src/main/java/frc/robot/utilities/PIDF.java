package frc.robot.utilities;

import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.ClosedLoopSlot;

public record PIDF(double p, double i, double d, double ff) {
  public PIDF {
    assert(p >= 0.0);
    assert(i >= 0.0);
    assert(d >= 0.0);
    assert(ff >= 0.0);
  }

  public PIDF(double p, double i, double d) {
    this(p, i, d, 0.0);
  }

  public void controllerSet(ClosedLoopConfig pidConfig, ClosedLoopSlot slot) {
    pidConfig.pidf(p, i, d, ff, slot);
  }

  public void controllerSet(ClosedLoopConfig pidConfig) {
    controllerSet(pidConfig, ClosedLoopSlot.kSlot0);
  }
}