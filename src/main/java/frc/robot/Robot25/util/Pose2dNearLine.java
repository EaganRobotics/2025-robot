package frc.robot.Robot25.util;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;

public final class Pose2dNearLine {
  /**
   * Returns whether the pose's translation is within 3 cm of the line segment defined by pointA and
   * pointB.
   *
   * @param pose the pose to check
   * @param pointA one endpoint of the segment
   * @param pointB the other endpoint of the segment
   * @param tolerance the tolerance for being near the segment
   * @return true if the distance is <= 3 cm, false otherwise.
   */
  public static boolean isNearSegment(Pose2d pose, Translation2d pointA, Translation2d pointB,
      Distance tolerance) {
    Translation2d p = pose.getTranslation();
    Translation2d ab = pointB.minus(pointA);
    // Compute squared length of ab
    double abSquared = ab.getX() * ab.getX() + ab.getY() * ab.getY();
    if (abSquared < 1E-6) { // avoid division by zero: segment is a point
      return p.getDistance(pointA) <= tolerance.in(Meters);
    }
    // Parameter t indicates where the projection falls relative to pointA (t=0) and
    // pointB (t=1)
    double t = (p.minus(pointA)).toVector().dot(ab.toVector()) / abSquared;
    // Clamp t between 0 and 1
    t = Math.max(0, Math.min(1, t));
    Translation2d projection = pointA.plus(ab.times(t));
    double distance = p.getDistance(projection);
    return distance <= tolerance.in(Meters);
  }
}
