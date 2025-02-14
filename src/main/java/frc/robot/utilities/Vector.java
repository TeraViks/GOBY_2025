package frc.robot.utilities;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Unbounded vector defined by a point and an angle. */
public record Vector(Translation2d p, Rotation2d angle) {
  /** Given a point known to be on the unbounded line coincident with the vector, determine
   *  whether the point is contained by the vector. */
  public boolean isContained(Translation2d p) {
    double x1 = this.p.getX();
    double y1 = this.p.getY();
    double x2 = p.getX();
    double y2 = p.getY();
    // Signs correspond between deltas and trig functions if contained.
    return (
      (x2 - x1) * angle.getCos() >= 0.0
      && (y2 - y1) * angle.getSin() >= 0.0
    );
  }
}