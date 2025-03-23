package frc.robot.Robot25.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.List;
import java.util.Optional;
import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.ConstraintsZone;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot25.subsystems.drive.Drive;

public final class PathPlanning {
  private static final Pose2d[] OUTER_REEF_POSITIONS =
      DriveCommands.makeReefPositions(Inches.of(12));
  private static final Pose2d[] INNER_REEF_POSITIONS =
      DriveCommands.makeReefPositions(Inches.of(0));

  public static final Command drivePathToClosestReef(Drive drive) {

    return Commands.defer(() -> {

      var poses = getClosestReefPosition(drive, Meters.of(100));

      if (poses.isEmpty()) {
        System.out.println("No nearest reef pose");
        return Commands.none();
      }

      var inner = poses.get().inner;
      var outer = poses.get().outer;

      var transformToInner = inner.minus(drive.getPose());

      var currentPoseAndHeading =
          new Pose2d(drive.getPose().getTranslation(), transformToInner.getRotation());

      // The rotation component of the pose should be the direction of travel. Do not
      // use holonomic rotation.
      List<Waypoint> waypoints =
          PathPlannerPath.waypointsFromPoses(currentPoseAndHeading, outer, inner);

      PathConstraints constraints = new PathConstraints(4.5, 6, 2 * Math.PI, 4 * Math.PI); // The
                                                                                           // constraints
                                                                                           // for
                                                                                           // this
      var endConstraints =
          new ConstraintsZone(0.7, 1, new PathConstraints(3, 2, 1 * Math.PI, 2 * Math.PI));
      // path.
      // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); //
      // You can also use unlimited constraints, only limited by motor torque and
      // nominal battery voltage

      // Create the path using the waypoints created above
      // new PathPlannerPath(waypoints, holonomicRotations, pointTowardsZones,
      // constraintZones, eventMarkers, globalConstraints, idealStartingState,
      // goalEndState, reversed)
      PathPlannerPath path = new PathPlannerPath(waypoints, List.of(), List.of(),
          List.of(endConstraints), List.of(), constraints, null, // The ideal starting state, this
                                                                 // is only relevant for pre-planned
                                                                 // paths, so can
                                                                 // be null for on-the-fly paths.
          new GoalEndState(0.1, inner.getRotation()), false);

      // Prevent the path from being flipped if the coordinates are already correct
      path.preventFlipping = true;

      return AutoBuilder.followPath(path);
    }, Set.of(drive));
  }

  private static Optional<Pose2dSequence> getClosestReefPosition(Drive drive, Distance radius) {
    Optional<Pose2dSequence> desiredPose = Optional.empty();
    Distance minDistance = Meters.of(1000000);
    for (int i = 0; i < OUTER_REEF_POSITIONS.length; i++) {
      Pose2d pose = OUTER_REEF_POSITIONS[i];
      double distance = drive.getPose().getTranslation().getDistance(pose.getTranslation());
      Distance distanceMeasure = Meters.of(distance);
      if (distanceMeasure.lte(radius) && distanceMeasure.lte(minDistance)) {
        minDistance = distanceMeasure;
        desiredPose =
            Optional.of(new Pose2dSequence(INNER_REEF_POSITIONS[i], OUTER_REEF_POSITIONS[i]));
      }
    }

    return desiredPose;
  };

  private static final class Pose2dSequence {
    Pose2d inner;
    Pose2d outer;

    public Pose2dSequence(Pose2d inner, Pose2d outer) {
      this.inner = inner;
      this.outer = outer;
    }

    private static final Pose2dSequence kZero = new Pose2dSequence(Pose2d.kZero, Pose2d.kZero);

  }
}
