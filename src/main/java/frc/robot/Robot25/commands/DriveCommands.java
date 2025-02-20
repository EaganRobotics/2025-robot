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
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.ShuffleBoardTabWrapper;
import frc.lib.tunables.TunableDouble;
import frc.robot.Robot25.subsystems.drive.Drive;
import frc.robot.Robot25.subsystems.drive.DriveConstants;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class DriveCommands {
  private static final double SLOW_MODE_MULTIPLIER = 0.5;
  private static final double DEADBAND = 0.1;
  // private static final double ANGLE_KP = 7.0;
  // private static final double ANGLE_KI = 0.0;
  // private static final double ANGLE_KD = 0.4;
  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;
  private static final double ANGLE_TOLERANCE = Degrees.of(0.05).in(Radians);
  private static final double POSITION_MAX_VELOCITY = 3.0;
  private static final double POSITION_MAX_ACCELERATION = 3.0;
  private static final double POSITION_TOLERANCE = Meters.of(0.0127).in(Meters);
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2


  // public static final TunableDouble ANGLE_KP =
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
      new LoggedNetworkNumber("/Tuning/positionKP", 0.001);
  public static final LoggedNetworkNumber POSITION_KI =
      new LoggedNetworkNumber("/Tuning/positionKI", 0);
  public static final LoggedNetworkNumber POSITION_KD =
      new LoggedNetworkNumber("/Tuning/positionKD", 0);


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
      //     (slowModeSupplier.getAsBoolean() ? SLOW_MODE_MULTIPLIER : 1.0);

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

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>
   * This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(() -> {
          velocitySamples.clear();
          voltageSamples.clear();
        }),

        // Allow modules to orient
        Commands.run(() -> {
          drive.runCharacterization(0.0);
        }, drive).withTimeout(FF_START_DELAY),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(() -> {
          double voltage = timer.get() * FF_RAMP_RATE;
          drive.runCharacterization(voltage);
          velocitySamples.add(drive.getFFCharacterizationVelocity());
          voltageSamples.add(voltage);
        }, drive)

            // When cancelled, calculate and print results
            .finallyDo(() -> {
              int n = velocitySamples.size();
              double sumX = 0.0;
              double sumY = 0.0;
              double sumXY = 0.0;
              double sumX2 = 0.0;
              for (int i = 0; i < n; i++) {
                sumX += velocitySamples.get(i);
                sumY += voltageSamples.get(i);
                sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
              }
              double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
              double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

              NumberFormat formatter = new DecimalFormat("#0.00000");
              System.out.println("********** Drive FF Characterization Results **********");
              System.out.println("\tkS: " + formatter.format(kS));
              System.out.println("\tkV: " + formatter.format(kV));
            }));
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(() -> {
              limiter.reset(0.0);
            }),

            // Turn in place, accelerating up to full speed
            Commands.run(() -> {
              double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
              drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
            }, drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(() -> {
              state.positions = drive.getWheelRadiusCharacterizationPositions();
              state.lastAngle = drive.getRotation();
              state.gyroDelta = 0.0;
            }),

            // Update gyro delta
            Commands.run(() -> {
              var rotation = drive.getRotation();
              state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
              state.lastAngle = rotation;
            })

                // When cancelled, calculate and print results
                .finallyDo(() -> {
                  double[] positions = drive.getWheelRadiusCharacterizationPositions();
                  double wheelDelta = 0.0;
                  for (int i = 0; i < 4; i++) {
                    wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                  }
                  double wheelRadius =
                      (state.gyroDelta * DriveConstants.DRIVE_BASE_RADIUS) / wheelDelta;

                  NumberFormat formatter = new DecimalFormat("#0.000");
                  System.out.println("********** Wheel Radius Characterization Results **********");
                  System.out.println("\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                  System.out
                      .println("\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                  System.out
                      .println("\tWheel Radius: " + formatter.format(wheelRadius) + " meters, "
                          + formatter.format(Units.metersToInches(wheelRadius)) + " inches");
                })));
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = Rotation2d.kZero;
    double gyroDelta = 0.0;
  }

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
      Logger.recordOutput("Snap/x/xPos", x);
      Logger.recordOutput("Snap/x/desiredXPos", desiredPosition.getX());
      Logger.recordOutput("Snap/x/currentXPos", drive.getPose().getX());
      Logger.recordOutput("Snap/y/yPos", y);
      Logger.recordOutput("Snap/y/desiredYPos", desiredPosition.getY());
      Logger.recordOutput("Snap/y/currentYPos", drive.getPose().getY());

      // Convert to field relative speeds & send command
      ChassisSpeeds speeds = new ChassisSpeeds(-x, -y, omega);
      drive.runVelocity(speeds);
    }, drive)

        // Reset PID controller when command starts
        .beforeStarting(() -> {
          angleController.reset(drive.getRotation().getRadians());
          // xController.reset(drive.getPose().getX());
          // yController.reset(drive.getPose().getY());
        });
  }
}
