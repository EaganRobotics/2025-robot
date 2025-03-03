// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.Robot25.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot25.subsystems.drive.Drive;
import java.util.Optional;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class DriveCommands {
  private static final double SLOW_MODE_MULTIPLIER = 0.5;
  private static final double DEADBAND = 0.1;
  // private static final double ANGLE_KP = 7.0;
  // private static final double ANGLE_KI = 0.0;
  // private static final double ANGLE_KD = 0.4;
  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;
  private static final double ANGLE_TOLERANCE = Degrees.of(1).in(Radians);
  private static final double POSITION_MAX_VELOCITY = 3.0;
  private static final double POSITION_MAX_ACCELERATION = 3.0;
  private static final double POSITION_TOLERANCE = Inches.of(1).in(Meters);

  /// Auto snap to position distance
  private static final Distance SNAPPY_RADIUS = Inches.of(12);

  private static final double INCHES_FROM_REEF = 16.75 + 11.757361;
  private static final double REEF_CENTER_X_INCHES = 176.745545;
  private static final double REEF_CENTER_Y_INCHES = 158.500907;

  private static final double Left_Loading_Station_X = 43.3071;
  private static final double Left_Loading_Station_Y = 275.591;

  private static final double Right_Loading_Station_X = 43.3071;
  private static final double Right_Loading_Station_Y = 29.52756;

  private static final Translation2d Left_Loading_Station =
      new Translation2d(Inches.of(Left_Loading_Station_X), Inches.of(Left_Loading_Station_Y));

  private static final Translation2d Right_Loading_Station =
      new Translation2d(Inches.of(Right_Loading_Station_X), Inches.of(Right_Loading_Station_Y));

  private static final Translation2d REEF_CENTER =
      new Translation2d(Inches.of(REEF_CENTER_X_INCHES), Inches.of(REEF_CENTER_Y_INCHES));

  public static Pose2d[] makeReefPositions(Distance reefOffset) {
    Transform2d REEF_BRANCH_TO_ROBOT = new Transform2d(
        Inches.of(-INCHES_FROM_REEF).minus(reefOffset), Inches.zero(), Rotation2d.kZero);
    return new Pose2d[] {
        new Pose2d(REEF_CENTER.plus(new Translation2d(Inches.of(-20.738000), Inches.of(6.482000))),
            Rotation2d.kZero).transformBy(REEF_BRANCH_TO_ROBOT),
        new Pose2d(REEF_CENTER.plus(new Translation2d(Inches.of(-20.738000), Inches.of(-6.482000))),
            Rotation2d.kZero).transformBy(REEF_BRANCH_TO_ROBOT),
        new Pose2d(
            REEF_CENTER.plus(new Translation2d(Inches.of(-15.982577), Inches.of(-14.718635))),
            Rotation2d.fromDegrees(60)).transformBy(REEF_BRANCH_TO_ROBOT),
        new Pose2d(REEF_CENTER.plus(new Translation2d(Inches.of(-4.755423), Inches.of(-21.200635))),
            Rotation2d.fromDegrees(60)).transformBy(REEF_BRANCH_TO_ROBOT),
        new Pose2d(REEF_CENTER.plus(new Translation2d(Inches.of(4.755423), Inches.of(-21.200635))),
            Rotation2d.fromDegrees(120)).transformBy(REEF_BRANCH_TO_ROBOT),
        new Pose2d(REEF_CENTER.plus(new Translation2d(Inches.of(15.982577), Inches.of(-14.718635))),
            Rotation2d.fromDegrees(120)).transformBy(REEF_BRANCH_TO_ROBOT),
        new Pose2d(REEF_CENTER.plus(new Translation2d(Inches.of(20.738000), Inches.of(-6.482000))),
            Rotation2d.fromDegrees(180)).transformBy(REEF_BRANCH_TO_ROBOT),
        new Pose2d(REEF_CENTER.plus(new Translation2d(Inches.of(20.738000), Inches.of(6.482000))),
            Rotation2d.fromDegrees(180)).transformBy(REEF_BRANCH_TO_ROBOT),
        new Pose2d(REEF_CENTER.plus(new Translation2d(Inches.of(15.982577), Inches.of(14.718635))),
            Rotation2d.fromDegrees(240)).transformBy(REEF_BRANCH_TO_ROBOT),
        new Pose2d(REEF_CENTER.plus(new Translation2d(Inches.of(4.755423), Inches.of(21.200635))),
            Rotation2d.fromDegrees(240)).transformBy(REEF_BRANCH_TO_ROBOT),
        new Pose2d(REEF_CENTER.plus(new Translation2d(Inches.of(-4.755423), Inches.of(21.200635))),
            Rotation2d.fromDegrees(300)).transformBy(REEF_BRANCH_TO_ROBOT),
        new Pose2d(REEF_CENTER.plus(new Translation2d(Inches.of(-15.982577), Inches.of(14.718635))),
            Rotation2d.fromDegrees(300)).transformBy(REEF_BRANCH_TO_ROBOT),

        // new Pose2d(Left_Loading_Station.plus(new Translation2d(Inches.of(0),
        // Inches.of(0))),
        // Rotation2d.fromDegrees(-50 + 180 + 180)),
        // new Pose2d(Right_Loading_Station.plus(new Translation2d(Inches.of(0),
        // Inches.of(0))),
        // Rotation2d.fromDegrees(130 + 90 + 180))

    };
  }

  private static final Pose2d[] OUTER_REEF_POSITIONS = makeReefPositions(Inches.of(12));
  private static final Pose2d[] INNER_REEF_POSITIONS = makeReefPositions(Inches.of(0));

  // public static final TunableDouble ANGLE_KP =/
  // new TunableDouble("ANGLE_KP", 7.0, "driver").setSpot(0, 0);
  // public static final TunableDouble ANGLE_KI =
  // new TunableDouble("ANGLE_KI", 0.0, "driver").setSpot(1, 0);
  // public static final TunableDouble ANGLE_KD =
  // new TunableDouble("ANGLE_KD", 0.4, "driver").setSpot(2, 0);

  public static final LoggedNetworkNumber ANGLE_KP =
      new LoggedNetworkNumber("/Tuning/angleKP", 7.0);
  public static final LoggedNetworkNumber ANGLE_KI =
      new LoggedNetworkNumber("/Tuning/angleKI", 0.0);
  public static final LoggedNetworkNumber ANGLE_KD =
      new LoggedNetworkNumber("/Tuning/angleKD", 0.4);

  public static final LoggedNetworkNumber POSITION_KP =
      new LoggedNetworkNumber("/Tuning/positionKP", 4);
  public static final LoggedNetworkNumber POSITION_KI =
      new LoggedNetworkNumber("/Tuning/positionKI", 0); // 1
  public static final LoggedNetworkNumber POSITION_KD =
      new LoggedNetworkNumber("/Tuning/positionKD", 0); // 1

  private DriveCommands() {}

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero)).getTranslation();
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(Drive drive, DoubleSupplier xSupplier,
      DoubleSupplier ySupplier, DoubleSupplier omegaSupplier) {

    // Create PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(ANGLE_KP.get(), ANGLE_KI.get(), ANGLE_KD.get(),
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);
    angleController.setTolerance(ANGLE_TOLERANCE);

    return Commands.run(() -> {
      angleController.setP(ANGLE_KP.get());
      angleController.setI(ANGLE_KI.get());
      angleController.setD(ANGLE_KD.get());

      // Get linear velocity
      Translation2d linearVelocity =
          getLinearVelocityFromJoysticks(-xSupplier.getAsDouble(), -ySupplier.getAsDouble());

      // Apply rotation deadband
      double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

      // Square rotation value for more precise control
      omega = Math.copySign(omega * omega, omega);

      // final double slowModeMultiplier =
      // (slowModeSupplier.getAsBoolean() ? SLOW_MODE_MULTIPLIER : 1.0);

      // No rotation
      if (Math.abs(omega) > 1E-6) {
        Logger.recordOutput("Rotation", "joystick");
        drive.setSnapToRotation(false);
        omega *= drive.getMaxAngularSpeedRadPerSec();
      } else if (drive.getSnapToRotation()) {
        omega = angleController.calculate(drive.getRotation().getRadians(),
            drive.getDesiredRotation().getRadians());
        if (angleController.atGoal()) {
          System.out.println("Snap to rotation complete");
          drive.setSnapToRotation(false);
        }
      } else {
        Logger.recordOutput("Rotation", "none");
        omega = 0.0;
      }

      final double maxSpeed = drive.getMaxLinearSpeedMetersPerSec();

      // Convert to field relative speeds & send command
      ChassisSpeeds speeds = new ChassisSpeeds(linearVelocity.getX() * maxSpeed,
          linearVelocity.getY() * maxSpeed, omega);
      drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drive.getRotation()));
    }, drive)

        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
  }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   */
  public static Command joystickDriveAtAngle(Drive drive, DoubleSupplier xSupplier,
      DoubleSupplier ySupplier, Supplier<Rotation2d> rotationSupplier) {

    // Create PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(ANGLE_KP.get(), ANGLE_KI.get(), ANGLE_KD.get(),
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construct command
    return Commands.run(() -> {
      // Get linear velocity
      Translation2d linearVelocity =
          getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

      // Calculate angular speed
      double omega = angleController.calculate(drive.getRotation().getRadians(),
          rotationSupplier.get().getRadians());

      // Convert to field relative speeds & send command
      ChassisSpeeds speeds =
          new ChassisSpeeds(linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
              linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(), omega);
      drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drive.getRotation()));
    }, drive)

        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
  }

  public static Command snapToRotation(Drive drive, Rotation2d rotation) {
    return Commands.runOnce(() -> {
      Logger.recordOutput("Rotation", "snap to rotation");
      drive.setDesiredRotation(rotation);
    });
  }

  public static Command keepRotationForward(Drive drive, DoubleSupplier xSupplier,
      DoubleSupplier ySupplier) {
    return Commands.run(() -> {
      double x = xSupplier.getAsDouble();
      double y = ySupplier.getAsDouble();

      var linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);

      if (linearMagnitude > 0.1) {
        Logger.recordOutput("Rotation", "robot forward");
        drive.setDesiredRotation(Rotation2d.fromRadians(Math.atan2(y, x)));
      }
    });
  }

  public static Command Snapper(Drive drive) {

    return Commands.defer(() -> {
      Pose2dSequence desiredPose =
          getClosestPosition(drive, Meters.of(1000)).orElse(Pose2dSequence.kZero);
      Logger.recordOutput("SnapperPose", desiredPose.outer);
      return snapToPosition(drive, desiredPose.outer)
          .andThen(snapToPosition(drive, desiredPose.inner));
    }, Set.of(drive));

  }

  public static Command AutoSnapper(Drive drive) {

    return Commands.defer(() -> {
      Pose2dSequence desiredPose =
          getClosestPosition(drive, Meters.of(1000)).orElse(Pose2dSequence.kZero);
      Logger.recordOutput("SnapperPose", desiredPose.outer);
      return snapToPosition(drive, desiredPose.inner);
    }, Set.of(drive));

  }

  public static Command AutoSnapperSource(Drive drive) {

    return Commands.defer(() -> {
      return snapToPosition(drive, new Pose2d(
          new Translation2d(Inches.of(Right_Loading_Station_X), Inches.of(Right_Loading_Station_Y)),
          Rotation2d.fromDegrees(55)));
    }, Set.of(drive));

  }

  private static final class Pose2dSequence {
    Pose2d inner;
    Pose2d outer;

    public Pose2dSequence(Pose2d inner, Pose2d outer) {
      this.inner = inner;
      this.outer = outer;
    }

    private static final Pose2dSequence kZero = new Pose2dSequence(Pose2d.kZero, Pose2d.kZero);

  }

  private static Optional<Pose2dSequence> getClosestPosition(Drive drive, Distance radius) {
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

  public static Command snapToPosition(Drive drive, Pose2d desiredPosition) {

    // Create PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(ANGLE_KP.get(), ANGLE_KI.get(), ANGLE_KD.get(),
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);
    angleController.setTolerance(ANGLE_TOLERANCE);

    ProfiledPIDController xController =
        new ProfiledPIDController(POSITION_KP.get(), POSITION_KI.get(), POSITION_KD.get(),
            new TrapezoidProfile.Constraints(POSITION_MAX_VELOCITY, POSITION_MAX_ACCELERATION));
    xController.setTolerance(POSITION_TOLERANCE);

    ProfiledPIDController yController =
        new ProfiledPIDController(POSITION_KP.get(), POSITION_KI.get(), POSITION_KD.get(),
            new TrapezoidProfile.Constraints(POSITION_MAX_VELOCITY, POSITION_MAX_ACCELERATION));
    yController.setTolerance(POSITION_TOLERANCE);

    return Commands.run(() -> {

      angleController.setP(ANGLE_KP.get());
      angleController.setI(ANGLE_KI.get());
      angleController.setD(ANGLE_KD.get());

      xController.setP(POSITION_KP.get());
      xController.setI(POSITION_KI.get());
      xController.setD(POSITION_KD.get());

      yController.setP(POSITION_KP.get());
      yController.setI(POSITION_KI.get());
      yController.setD(POSITION_KD.get());

      var x = xController.calculate(drive.getPose().getX(), desiredPosition.getX());

      var y = yController.calculate(drive.getPose().getY(), desiredPosition.getY());

      var omega = angleController.calculate(drive.getRotation().getRadians(),
          desiredPosition.getRotation().getRadians());

      Logger.recordOutput("Snap/omega", omega);
      Logger.recordOutput("Snap/x/xDiff", x);
      Logger.recordOutput("Snap/x/desiredXPos", desiredPosition.getX());
      Logger.recordOutput("Snap/x/currentXPos", drive.getPose().getX());
      Logger.recordOutput("Snap/y/yDiff", y);
      Logger.recordOutput("Snap/y/desiredYPos", desiredPosition.getY());
      Logger.recordOutput("Snap/y/currentYPos", drive.getPose().getY());
      Logger.recordOutput("Snap/desiredPos", desiredPosition);

      // Convert to field relative speeds & send command
      ChassisSpeeds speeds = new ChassisSpeeds(x, y, omega);
      drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drive.getRotation()));

    }, drive)

        // Reset PID controller when command starts
        .beforeStarting(() -> {
          angleController.reset(drive.getRotation().getRadians());
          xController.reset(drive.getPose().getX());
          yController.reset(drive.getPose().getY());
        }).until(() -> angleController.atGoal() && xController.atGoal() && yController.atGoal());
  }

  public static Command joystickDriveAssist(Drive drive, DoubleSupplier xSupplier,
      DoubleSupplier ySupplier, DoubleSupplier omegaSupplier, BooleanSupplier snapSupplier,
      BooleanSupplier slowModeSupplier) {

    // Create PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(ANGLE_KP.get(), ANGLE_KI.get(), ANGLE_KD.get(),
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);
    angleController.setTolerance(ANGLE_TOLERANCE);

    ProfiledPIDController xController =
        new ProfiledPIDController(POSITION_KP.get(), POSITION_KI.get(), POSITION_KD.get(),
            new TrapezoidProfile.Constraints(POSITION_MAX_VELOCITY, POSITION_MAX_ACCELERATION));
    xController.setTolerance(POSITION_TOLERANCE);

    ProfiledPIDController yController =
        new ProfiledPIDController(POSITION_KP.get(), POSITION_KI.get(), POSITION_KD.get(),
            new TrapezoidProfile.Constraints(POSITION_MAX_VELOCITY, POSITION_MAX_ACCELERATION));
    yController.setTolerance(POSITION_TOLERANCE);

    return Commands.run(() -> {

      angleController.setP(ANGLE_KP.get());
      angleController.setI(ANGLE_KI.get());
      angleController.setD(ANGLE_KD.get());

      xController.setP(POSITION_KP.get());
      xController.setI(POSITION_KI.get());
      xController.setD(POSITION_KD.get());

      yController.setP(POSITION_KP.get());
      yController.setI(POSITION_KI.get());
      yController.setD(POSITION_KD.get());

      Logger.recordOutput("InnerReefPositions", DriveCommands.INNER_REEF_POSITIONS);
      Logger.recordOutput("OuterReefPositions", DriveCommands.OUTER_REEF_POSITIONS);

      final double slowModeMultiplier =
          (slowModeSupplier.getAsBoolean() ? SLOW_MODE_MULTIPLIER : 1.0);

      // Get linear velocity
      Translation2d linearVelocity =
          getLinearVelocityFromJoysticks(-xSupplier.getAsDouble(), -ySupplier.getAsDouble());

      // Apply rotation deadband
      double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

      final double maxSpeed = drive.getMaxLinearSpeedMetersPerSec();

      double x = linearVelocity.getX() * maxSpeed * slowModeMultiplier;
      double y = linearVelocity.getY() * maxSpeed * slowModeMultiplier;

      // Square rotation value for more precise control
      omega = Math.copySign(omega * omega, omega);

      omega *= drive.getMaxAngularSpeedRadPerSec();
      if ((Math.abs(omega) > 1E-6) || (Math.abs(x) > 1E-6) || (Math.abs(y) > 1E-6)) {
        Logger.recordOutput("DriveState", "Driver");
        Logger.recordOutput("Snap/desiredPos", new Pose2d(-50, -50, Rotation2d.kZero));
      } else if (snapSupplier.getAsBoolean()) {
        Optional<Pose2dSequence> closestOptionalPose = getClosestPosition(drive, SNAPPY_RADIUS);

        if (closestOptionalPose.isPresent()) {
          Pose2dSequence closestPoseSequence = closestOptionalPose.orElse(Pose2dSequence.kZero);
          Pose2d closestPose = closestPoseSequence.inner;
          Logger.recordOutput("DriveState", "Robot");
          Logger.recordOutput("Snap/desiredPos", closestPose);
          if (angleController.atGoal() && xController.atGoal() && yController.atGoal()) {
            x = 0;
            y = 0;
            omega = 0;
          } else {
            x = xController.calculate(drive.getPose().getX(), closestPose.getX());

            y = yController.calculate(drive.getPose().getY(), closestPose.getY());

            omega = angleController.calculate(drive.getRotation().getRadians(),
                closestPose.getRotation().getRadians());
          }

        }
      }

      Logger.recordOutput("Snap/omega", omega);
      Logger.recordOutput("Snap/x/xDiff", x);
      Logger.recordOutput("Snap/x/currentXPos", drive.getPose().getX());
      Logger.recordOutput("Snap/y/yDiff", y);
      Logger.recordOutput("Snap/y/currentYPos", drive.getPose().getY());

      // Convert to field relative speeds & send command
      ChassisSpeeds speeds = new ChassisSpeeds(x, y, omega);
      drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drive.getRotation()));

    }, drive)

        // Reset PID controller when command starts
        .beforeStarting(() ->

        {
          angleController.reset(drive.getRotation().getRadians());
          xController.reset(drive.getPose().getX());
          yController.reset(drive.getPose().getY());
        });
  }
}
