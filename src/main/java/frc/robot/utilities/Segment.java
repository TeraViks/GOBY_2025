package frc.robot.utilities;

import java.util.Optional;

import edu.wpi.first.math.geometry.Translation2d;

/** Line segment defined by two points. */
public record Segment(Translation2d p1, Translation2d p2) {
  /* Compute the intersection of unbounded lines coincident with the segments. */
  private Optional<Translation2d> lineLineIntersection(Segment s) {
    // Use the formula at
    // https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection#Given_two_points_on_each_line
    // to compute the intersection of the two infinite lines which contain the segments.
    double x1 = this.p1.getX();
    double y1 = this.p1.getY();
    double x2 = this.p2.getX();
    double y2 = this.p2.getY();
    double x3 = s.p1.getX();
    double y3 = s.p1.getY();
    double x4 = s.p2.getX();
    double y4 = s.p2.getY();
    double d = (x1 - x2)*(y3 - y4) - (y1 - y2)*(x3 - x4);
    if (d == 0.0) {
      // Coincident or parallel.
      return Optional.empty();
    }
    double n1 = x1*y2 - y1*x2;
    double n2 = x3*y4 - y3*x4;
    double x = (n1*(x3 - x4) - (x1 - x2)*n2) / d;
    double y = (n1*(y3 - y4) - (y1 - y2)*n2) / d;
    return Optional.of(new Translation2d(x, y));
  }

  /** Given a point known to be on the unbounded line coincident with the segment, determine
   *  whether the point is contained by the segment. */
  public boolean isContained(Translation2d p) {
    double x1 = this.p1.getX();
    double y1 = this.p1.getY();
    double x2 = this.p2.getX();
    double y2 = this.p2.getY();
    double x = p.getX();
    double y = p.getY();
    return (x >= Math.min(x1, x2) && x <= Math.max(x1, x2)
      && y >= Math.min(y1, y2) && y <= Math.max(y1, y2));
  }

  /** Compute the intersection of two line segments. */
  public Optional<Translation2d> intersection(Segment s) {
    Optional<Translation2d> pOpt = lineLineIntersection(s);
    if (pOpt.isEmpty()) {
      return pOpt;
    }
    Translation2d p = pOpt.get();
    // Check whether intersection is contained by both segments.
    if (!isContained(p) || !s.isContained(p)) {
      return Optional.empty();
    }
    return pOpt;
  }

  /** Compute the intersection of a segment and an unbounded vector. */
  public Optional<Translation2d> intersection(Vector v) {
    Translation2d p1 = v.p();
    Translation2d p2 = p1.plus(new Translation2d(v.angle().getCos(), v.angle().getSin()));
    Segment s = new Segment(p1, p2);
    Optional<Translation2d> pOpt = lineLineIntersection(s);
    if (pOpt.isEmpty()) {
      return pOpt;
    }
    Translation2d p = pOpt.get();
    // Check whether the intersection is contained by the segment and the vector.
    if (!isContained(p) || !v.isContained(p)) {
      return Optional.empty();
    }
    return pOpt;
  }
}