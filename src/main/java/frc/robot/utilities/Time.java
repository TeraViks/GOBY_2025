package frc.robot.utilities;

import edu.wpi.first.wpilibj.RobotController;

public final class Time {
  /** Get time since robot start in microseconds. */
  public static long getTimeMicroseconds() {
    return RobotController.getFPGATime();
  }

  /** Get time since robot start in seconds, with microsecond resolution. */
  public static double getTimeSeconds() {
    return (double)getTimeMicroseconds() / 1_000_000.0;
  }
}