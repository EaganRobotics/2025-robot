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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.tunables.LoggedTunableNumber;
import frc.robot.Robot25.subsystems.drive.Drive;

import java.util.HashSet;
import java.util.Optional;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveCommands {
  private static final double SLOW_MODE_MULTIPLIER = 0.2;
  private static final double DEADBAND = 0.05;
  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;
  private static final double ANGLE_TOLERANCE = Degrees.of(1).in(Radians);
  private static final double POSITION_MAX_VELOCITY = 4.5;
  private static final double POSITION_MAX_ACCELERATION = 6;
  private static final double POSITION_TOLERANCE = Inches.of(1).in(Meters);

  /// Auto snap to position distance
  private static final Distance SNAPPY_RADIUS = Inches.of(12);

  private static final double INCHES_FROM_REEF = 16.75 + 11.757361 - 0.5;
  private static final double REEF_CENTER_X_INCHES = 176.745545;
  private static final double REEF_CENTER_Y_INCHES = 158.500907;

  private static final double INCHES_FROM_ALGAE = 16.75 + 11.757361 - 0.5 + 20;

  private static final double BLUE_BARGE_X = 290;
  private static final double LEFT_BLUE_BARGE_Y = 285;
  private static final double MIDDLE_BLUE_BARGE_Y = 238;
  private static final double RIGHT_BLUE_BARGE_Y = 190;

  private static final double RED_BARGE_X = 390;
  private static final double LEFT_RED_BARGE_Y = 120;
  private static final double MIDDLE_RED_BARGE_Y = 73;
  private static final double RIGHT_RED_BARGE_Y = 25;

  private static final double Left_Loading_Station_X = 43.3071;
  private static final double Left_Loading_Station_Y = 275.591;

  private static final double Right_Loading_Station_X = 43.3071;
  private static final double Right_Loading_Station_Y = 29.52756;

  private static final Translation2d BLUE_REEF_CENTER =
      new Translation2d(Inches.of(REEF_CENTER_X_INCHES), Inches.of(REEF_CENTER_Y_INCHES));
  private static final Translation2d RED_REEF_CENTER =
      new Translation2d(Inches.of(REEF_CENTER_X_INCHES + 337.385), Inches.of(REEF_CENTER_Y_INCHES));

  // private static final Distance BARGE_LINE = Inches.of(50);

  public static Pose2d[] makeReefPositions(Distance reefOffset) {
    Transform2d REEF_BRANCH_TO_ROBOT = new Transform2d(
        Inches.of(-INCHES_FROM_REEF).minus(reefOffset), Inches.zero(), Rotation2d.kZero);
    return new Pose2d[] {
        new Pose2d(
            BLUE_REEF_CENTER.plus(new Translation2d(Inches.of(-20.738000), Inches.of(6.482000))),
            Rotation2d.kZero).transformBy(REEF_BRANCH_TO_ROBOT),
        new Pose2d(
            BLUE_REEF_CENTER.plus(new Translation2d(Inches.of(-20.738000), Inches.of(-6.482000))),
            Rotation2d.kZero).transformBy(REEF_BRANCH_TO_ROBOT),
        new Pose2d(
            BLUE_REEF_CENTER.plus(new Translation2d(Inches.of(-15.982577), Inches.of(-14.718635))),
            Rotation2d.fromDegrees(60)).transformBy(REEF_BRANCH_TO_ROBOT),
        new Pose2d(
            BLUE_REEF_CENTER.plus(new Translation2d(Inches.of(-4.755423), Inches.of(-21.200635))),
            Rotation2d.fromDegrees(60)).transformBy(REEF_BRANCH_TO_ROBOT),
        new Pose2d(
            BLUE_REEF_CENTER.plus(new Translation2d(Inches.of(4.755423), Inches.of(-21.200635))),
            Rotation2d.fromDegrees(120)).transformBy(REEF_BRANCH_TO_ROBOT),
        new Pose2d(
            BLUE_REEF_CENTER.plus(new Translation2d(Inches.of(15.982577), Inches.of(-14.718635))),
            Rotation2d.fromDegrees(120)).transformBy(REEF_BRANCH_TO_ROBOT),
        new Pose2d(
            BLUE_REEF_CENTER.plus(new Translation2d(Inches.of(20.738000), Inches.of(-6.482000))),
            Rotation2d.fromDegrees(180)).transformBy(REEF_BRANCH_TO_ROBOT),
        new Pose2d(
            BLUE_REEF_CENTER.plus(new Translation2d(Inches.of(20.738000), Inches.of(6.482000))),
            Rotation2d.fromDegrees(180)).transformBy(REEF_BRANCH_TO_ROBOT),
        new Pose2d(
            BLUE_REEF_CENTER.plus(new Translation2d(Inches.of(15.982577), Inches.of(14.718635))),
            Rotation2d.fromDegrees(240)).transformBy(REEF_BRANCH_TO_ROBOT),
        new Pose2d(
            BLUE_REEF_CENTER.plus(new Translation2d(Inches.of(4.755423), Inches.of(21.200635))),
            Rotation2d.fromDegrees(240)).transformBy(REEF_BRANCH_TO_ROBOT),
        new Pose2d(
            BLUE_REEF_CENTER.plus(new Translation2d(Inches.of(-4.755423), Inches.of(21.200635))),
            Rotation2d.fromDegrees(300)).transformBy(REEF_BRANCH_TO_ROBOT),
        new Pose2d(
            BLUE_REEF_CENTER.plus(new Translation2d(Inches.of(-15.982577), Inches.of(14.718635))),
            Rotation2d.fromDegrees(300)).transformBy(REEF_BRANCH_TO_ROBOT),

        new Pose2d(
            RED_REEF_CENTER.plus(new Translation2d(Inches.of(-20.738000), Inches.of(6.482000))),
            Rotation2d.kZero).transformBy(REEF_BRANCH_TO_ROBOT),
        new Pose2d(
            RED_REEF_CENTER.plus(new Translation2d(Inches.of(-20.738000), Inches.of(-6.482000))),
            Rotation2d.kZero).transformBy(REEF_BRANCH_TO_ROBOT),
        new Pose2d(
            RED_REEF_CENTER.plus(new Translation2d(Inches.of(-15.982577), Inches.of(-14.718635))),
            Rotation2d.fromDegrees(60)).transformBy(REEF_BRANCH_TO_ROBOT),
        new Pose2d(
            RED_REEF_CENTER.plus(new Translation2d(Inches.of(-4.755423), Inches.of(-21.200635))),
            Rotation2d.fromDegrees(60)).transformBy(REEF_BRANCH_TO_ROBOT),
        new Pose2d(
            RED_REEF_CENTER.plus(new Translation2d(Inches.of(4.755423), Inches.of(-21.200635))),
            Rotation2d.fromDegrees(120)).transformBy(REEF_BRANCH_TO_ROBOT),
        new Pose2d(
            RED_REEF_CENTER.plus(new Translation2d(Inches.of(15.982577), Inches.of(-14.718635))),
            Rotation2d.fromDegrees(120)).transformBy(REEF_BRANCH_TO_ROBOT),
        new Pose2d(
            RED_REEF_CENTER.plus(new Translation2d(Inches.of(20.738000), Inches.of(-6.482000))),
            Rotation2d.fromDegrees(180)).transformBy(REEF_BRANCH_TO_ROBOT),
        new Pose2d(
            RED_REEF_CENTER.plus(new Translation2d(Inches.of(20.738000), Inches.of(6.482000))),
            Rotation2d.fromDegrees(180)).transformBy(REEF_BRANCH_TO_ROBOT),
        new Pose2d(
            RED_REEF_CENTER.plus(new Translation2d(Inches.of(15.982577), Inches.of(14.718635))),
            Rotation2d.fromDegrees(240)).transformBy(REEF_BRANCH_TO_ROBOT),
        new Pose2d(
            RED_REEF_CENTER.plus(new Translation2d(Inches.of(4.755423), Inches.of(21.200635))),
            Rotation2d.fromDegrees(240)).transformBy(REEF_BRANCH_TO_ROBOT),
        new Pose2d(
            RED_REEF_CENTER.plus(new Translation2d(Inches.of(-4.755423), Inches.of(21.200635))),
            Rotation2d.fromDegrees(300)).transformBy(REEF_BRANCH_TO_ROBOT),
        new Pose2d(
            RED_REEF_CENTER.plus(new Translation2d(Inches.of(-15.982577), Inches.of(14.718635))),
            Rotation2d.fromDegrees(300)).transformBy(REEF_BRANCH_TO_ROBOT),};
  }

  public static Pose2d[] makeRightReefPositions(Distance reefOffset) {
    Transform2d REEF_BRANCH_TO_ROBOT = new Transform2d(
        Inches.of(-INCHES_FROM_REEF).minus(reefOffset), Inches.zero(), Rotation2d.kZero);
    return new Pose2d[] {
        new Pose2d(
            BLUE_REEF_CENTER.plus(new Translation2d(Inches.of(-20.738000), Inches.of(-6.482000))),
            Rotation2d.kZero).transformBy(REEF_BRANCH_TO_ROBOT),
        new Pose2d(
            BLUE_REEF_CENTER.plus(new Translation2d(Inches.of(-4.755423), Inches.of(-21.200635))),
            Rotation2d.fromDegrees(60)).transformBy(REEF_BRANCH_TO_ROBOT),
        new Pose2d(
            BLUE_REEF_CENTER.plus(new Translation2d(Inches.of(15.982577), Inches.of(-14.718635))),
            Rotation2d.fromDegrees(120)).transformBy(REEF_BRANCH_TO_ROBOT),
        new Pose2d(
            BLUE_REEF_CENTER.plus(new Translation2d(Inches.of(20.738000), Inches.of(6.482000))),
            Rotation2d.fromDegrees(180)).transformBy(REEF_BRANCH_TO_ROBOT),
        new Pose2d(
            BLUE_REEF_CENTER.plus(new Translation2d(Inches.of(4.755423), Inches.of(21.200635))),
            Rotation2d.fromDegrees(240)).transformBy(REEF_BRANCH_TO_ROBOT),
        new Pose2d(
            BLUE_REEF_CENTER.plus(new Translation2d(Inches.of(-15.982577), Inches.of(14.718635))),
            Rotation2d.fromDegrees(300)).transformBy(REEF_BRANCH_TO_ROBOT),

        new Pose2d(
            RED_REEF_CENTER.plus(new Translation2d(Inches.of(-20.738000), Inches.of(-6.482000))),
            Rotation2d.kZero).transformBy(REEF_BRANCH_TO_ROBOT),
        new Pose2d(
            RED_REEF_CENTER.plus(new Translation2d(Inches.of(-4.755423), Inches.of(-21.200635))),
            Rotation2d.fromDegrees(60)).transformBy(REEF_BRANCH_TO_ROBOT),
        new Pose2d(
            RED_REEF_CENTER.plus(new Translation2d(Inches.of(15.982577), Inches.of(-14.718635))),
            Rotation2d.fromDegrees(120)).transformBy(REEF_BRANCH_TO_ROBOT),
        new Pose2d(
            RED_REEF_CENTER.plus(new Translation2d(Inches.of(20.738000), Inches.of(6.482000))),
            Rotation2d.fromDegrees(180)).transformBy(REEF_BRANCH_TO_ROBOT),
        new Pose2d(
            RED_REEF_CENTER.plus(new Translation2d(Inches.of(4.755423), Inches.of(21.200635))),
            Rotation2d.fromDegrees(240)).transformBy(REEF_BRANCH_TO_ROBOT),
        new Pose2d(
            RED_REEF_CENTER.plus(new Translation2d(Inches.of(-15.982577), Inches.of(14.718635))),
            Rotation2d.fromDegrees(300)).transformBy(REEF_BRANCH_TO_ROBOT),};
  }

  public static Pose2d[] makeLeftReefPositions(Distance reefOffset) {
    Transform2d REEF_BRANCH_TO_ROBOT = new Transform2d(
        Inches.of(-INCHES_FROM_REEF).minus(reefOffset), Inches.zero(), Rotation2d.kZero);
    return new Pose2d[] {
        new Pose2d(
            BLUE_REEF_CENTER.plus(new Translation2d(Inches.of(-20.738000), Inches.of(6.482000))),
            Rotation2d.kZero).transformBy(REEF_BRANCH_TO_ROBOT),
        new Pose2d(
            BLUE_REEF_CENTER.plus(new Translation2d(Inches.of(-15.982577), Inches.of(-14.718635))),
            Rotation2d.fromDegrees(60)).transformBy(REEF_BRANCH_TO_ROBOT),
        new Pose2d(
            BLUE_REEF_CENTER.plus(new Translation2d(Inches.of(4.755423), Inches.of(-21.200635))),
            Rotation2d.fromDegrees(120)).transformBy(REEF_BRANCH_TO_ROBOT),
        new Pose2d(
            BLUE_REEF_CENTER.plus(new Translation2d(Inches.of(20.738000), Inches.of(-6.482000))),
            Rotation2d.fromDegrees(180)).transformBy(REEF_BRANCH_TO_ROBOT),
        new Pose2d(
            BLUE_REEF_CENTER.plus(new Translation2d(Inches.of(15.982577), Inches.of(14.718635))),
            Rotation2d.fromDegrees(240)).transformBy(REEF_BRANCH_TO_ROBOT),
        new Pose2d(
            BLUE_REEF_CENTER.plus(new Translation2d(Inches.of(-4.755423), Inches.of(21.200635))),
            Rotation2d.fromDegrees(300)).transformBy(REEF_BRANCH_TO_ROBOT),

        new Pose2d(
            RED_REEF_CENTER.plus(new Translation2d(Inches.of(-20.738000), Inches.of(6.482000))),
            Rotation2d.kZero).transformBy(REEF_BRANCH_TO_ROBOT),
        new Pose2d(
            RED_REEF_CENTER.plus(new Translation2d(Inches.of(-15.982577), Inches.of(-14.718635))),
            Rotation2d.fromDegrees(60)).transformBy(REEF_BRANCH_TO_ROBOT),
        new Pose2d(
            RED_REEF_CENTER.plus(new Translation2d(Inches.of(4.755423), Inches.of(-21.200635))),
            Rotation2d.fromDegrees(120)).transformBy(REEF_BRANCH_TO_ROBOT),
        new Pose2d(
            RED_REEF_CENTER.plus(new Translation2d(Inches.of(20.738000), Inches.of(-6.482000))),
            Rotation2d.fromDegrees(180)).transformBy(REEF_BRANCH_TO_ROBOT),
        new Pose2d(
            RED_REEF_CENTER.plus(new Translation2d(Inches.of(15.982577), Inches.of(14.718635))),
            Rotation2d.fromDegrees(240)).transformBy(REEF_BRANCH_TO_ROBOT),
        new Pose2d(
            RED_REEF_CENTER.plus(new Translation2d(Inches.of(-4.755423), Inches.of(21.200635))),
            Rotation2d.fromDegrees(300)).transformBy(REEF_BRANCH_TO_ROBOT),};
  }

  public static Pose2d[] makeLLReefPositions(Distance reefOffset) {
    Transform2d REEF_BRANCH_TO_ROBOT = new Transform2d(
        Inches.of(-INCHES_FROM_REEF).minus(reefOffset), Inches.zero(), Rotation2d.kZero);
    return new Pose2d[] {
        new Pose2d(
            BLUE_REEF_CENTER.plus(new Translation2d(Inches.of(-20.738000), Inches.of(6.482000))),
            Rotation2d.kZero).transformBy(REEF_BRANCH_TO_ROBOT),
        new Pose2d(
            BLUE_REEF_CENTER.plus(new Translation2d(Inches.of(-20.738000), Inches.of(-6.482000))),
            Rotation2d.kZero).transformBy(REEF_BRANCH_TO_ROBOT),
        new Pose2d(
            BLUE_REEF_CENTER.plus(new Translation2d(Inches.of(-15.982577), Inches.of(-14.718635))),
            Rotation2d.fromDegrees(60)).transformBy(REEF_BRANCH_TO_ROBOT),
        new Pose2d(
            BLUE_REEF_CENTER.plus(new Translation2d(Inches.of(-4.755423), Inches.of(-21.200635))),
            Rotation2d.fromDegrees(60)).transformBy(REEF_BRANCH_TO_ROBOT),
        new Pose2d(
            BLUE_REEF_CENTER.plus(new Translation2d(Inches.of(4.755423), Inches.of(-21.200635))),
            Rotation2d.fromDegrees(120)).transformBy(REEF_BRANCH_TO_ROBOT),
        new Pose2d(
            BLUE_REEF_CENTER.plus(new Translation2d(Inches.of(15.982577), Inches.of(-14.718635))),
            Rotation2d.fromDegrees(120)).transformBy(REEF_BRANCH_TO_ROBOT),
        new Pose2d(
            BLUE_REEF_CENTER.plus(new Translation2d(Inches.of(15.982577), Inches.of(14.718635))),
            Rotation2d.fromDegrees(240)).transformBy(REEF_BRANCH_TO_ROBOT),
        new Pose2d(
            BLUE_REEF_CENTER.plus(new Translation2d(Inches.of(4.755423), Inches.of(21.200635))),
            Rotation2d.fromDegrees(240)).transformBy(REEF_BRANCH_TO_ROBOT),
        new Pose2d(
            BLUE_REEF_CENTER.plus(new Translation2d(Inches.of(-4.755423), Inches.of(21.200635))),
            Rotation2d.fromDegrees(300)).transformBy(REEF_BRANCH_TO_ROBOT),
        new Pose2d(
            BLUE_REEF_CENTER.plus(new Translation2d(Inches.of(-15.982577), Inches.of(14.718635))),
            Rotation2d.fromDegrees(300)).transformBy(REEF_BRANCH_TO_ROBOT),


        new Pose2d(
            RED_REEF_CENTER.plus(new Translation2d(Inches.of(-15.982577), Inches.of(-14.718635))),
            Rotation2d.fromDegrees(60)).transformBy(REEF_BRANCH_TO_ROBOT),
        new Pose2d(
            RED_REEF_CENTER.plus(new Translation2d(Inches.of(-4.755423), Inches.of(-21.200635))),
            Rotation2d.fromDegrees(60)).transformBy(REEF_BRANCH_TO_ROBOT),
        new Pose2d(
            RED_REEF_CENTER.plus(new Translation2d(Inches.of(4.755423), Inches.of(-21.200635))),
            Rotation2d.fromDegrees(120)).transformBy(REEF_BRANCH_TO_ROBOT),
        new Pose2d(
            RED_REEF_CENTER.plus(new Translation2d(Inches.of(15.982577), Inches.of(-14.718635))),
            Rotation2d.fromDegrees(120)).transformBy(REEF_BRANCH_TO_ROBOT),
        new Pose2d(
            RED_REEF_CENTER.plus(new Translation2d(Inches.of(20.738000), Inches.of(-6.482000))),
            Rotation2d.fromDegrees(180)).transformBy(REEF_BRANCH_TO_ROBOT),
        new Pose2d(
            RED_REEF_CENTER.plus(new Translation2d(Inches.of(20.738000), Inches.of(6.482000))),
            Rotation2d.fromDegrees(180)).transformBy(REEF_BRANCH_TO_ROBOT),
        new Pose2d(
            RED_REEF_CENTER.plus(new Translation2d(Inches.of(15.982577), Inches.of(14.718635))),
            Rotation2d.fromDegrees(240)).transformBy(REEF_BRANCH_TO_ROBOT),
        new Pose2d(
            RED_REEF_CENTER.plus(new Translation2d(Inches.of(4.755423), Inches.of(21.200635))),
            Rotation2d.fromDegrees(240)).transformBy(REEF_BRANCH_TO_ROBOT),
        new Pose2d(
            RED_REEF_CENTER.plus(new Translation2d(Inches.of(-4.755423), Inches.of(21.200635))),
            Rotation2d.fromDegrees(300)).transformBy(REEF_BRANCH_TO_ROBOT),
        new Pose2d(
            RED_REEF_CENTER.plus(new Translation2d(Inches.of(-15.982577), Inches.of(14.718635))),
            Rotation2d.fromDegrees(300)).transformBy(REEF_BRANCH_TO_ROBOT),

    };
  }


  public static Pose2d[] makeAlgaePositions(Distance algaeOffset) {
    Transform2d FRONT_ALGAE_TO_ROBOT = new Transform2d(
        Inches.of(-INCHES_FROM_ALGAE).minus(algaeOffset), Inches.zero(), Rotation2d.kZero);
    Transform2d FRONT_RIGHT_TO_ROBOT = new Transform2d(
        Inches.of(-INCHES_FROM_ALGAE).minus(algaeOffset), Inches.zero(), Rotation2d.kZero);
    Transform2d FRONT_LEFT_TO_ROBOT = new Transform2d(
        Inches.of(-INCHES_FROM_ALGAE).minus(algaeOffset), Inches.zero(), Rotation2d.kZero);
    Transform2d BACK_ALGAE_TO_ROBOT = new Transform2d(
        Inches.of(-INCHES_FROM_ALGAE).minus(algaeOffset), Inches.zero(), Rotation2d.kZero);
    Transform2d BACK_RIGHT_TO_ROBOT = new Transform2d(
        Inches.of(-INCHES_FROM_ALGAE).minus(algaeOffset), Inches.zero(), Rotation2d.kZero);
    Transform2d BACK_LEFT_TO_ROBOT = new Transform2d(
        Inches.of(-INCHES_FROM_ALGAE).minus(algaeOffset), Inches.zero(), Rotation2d.kZero);

    return new Pose2d[] {
        new Pose2d(BLUE_REEF_CENTER, Rotation2d.kZero).transformBy(FRONT_ALGAE_TO_ROBOT),
        new Pose2d(BLUE_REEF_CENTER, Rotation2d.fromDegrees(60)).transformBy(FRONT_RIGHT_TO_ROBOT),
        new Pose2d(BLUE_REEF_CENTER, Rotation2d.fromDegrees(300)).transformBy(FRONT_LEFT_TO_ROBOT),
        new Pose2d(BLUE_REEF_CENTER, Rotation2d.fromDegrees(180)).transformBy(BACK_ALGAE_TO_ROBOT),
        new Pose2d(BLUE_REEF_CENTER, Rotation2d.fromDegrees(120)).transformBy(BACK_RIGHT_TO_ROBOT),
        new Pose2d(BLUE_REEF_CENTER, Rotation2d.fromDegrees(240)).transformBy(BACK_LEFT_TO_ROBOT),

        new Pose2d(RED_REEF_CENTER, Rotation2d.kZero).transformBy(FRONT_ALGAE_TO_ROBOT),
        new Pose2d(RED_REEF_CENTER, Rotation2d.fromDegrees(60)).transformBy(FRONT_RIGHT_TO_ROBOT),
        new Pose2d(RED_REEF_CENTER, Rotation2d.fromDegrees(300)).transformBy(FRONT_LEFT_TO_ROBOT),
        new Pose2d(RED_REEF_CENTER, Rotation2d.fromDegrees(180)).transformBy(BACK_ALGAE_TO_ROBOT),
        new Pose2d(RED_REEF_CENTER, Rotation2d.fromDegrees(120)).transformBy(BACK_RIGHT_TO_ROBOT),
        new Pose2d(RED_REEF_CENTER, Rotation2d.fromDegrees(240)).transformBy(BACK_LEFT_TO_ROBOT),};
  }

  public static Pose2d[] makeSourcePositions() {
    return new Pose2d[] {

        new Pose2d(Inches.of(Right_Loading_Station_X + 5.5),
            Inches.of(Right_Loading_Station_Y + 5.5), Rotation2d.fromDegrees(55)),
        new Pose2d(Inches.of(Left_Loading_Station_X - 1.5), Inches.of(Left_Loading_Station_Y + 1.5),
            Rotation2d.fromDegrees(-55)),
        new Pose2d(Inches.of(Right_Loading_Station_X + 5.5 + 623.825 - 8 - 6 - 2),
            Inches.of(Right_Loading_Station_Y + 5.5 + 3 + 6 + 2), Rotation2d.fromDegrees(125)),
        new Pose2d(Inches.of(Left_Loading_Station_X - 1.5 + 623.825 - 10),
            Inches.of(Left_Loading_Station_Y + 1.5 - 4.5), Rotation2d.fromDegrees(-125))};

  }

  public static Pose2d[] makeBargePositions(Distance bargeOffset) {

    return new Pose2d[] {
        new Pose2d(Inches.of(BLUE_BARGE_X), Inches.of(LEFT_BLUE_BARGE_Y), Rotation2d.kZero),
        new Pose2d(Inches.of(BLUE_BARGE_X), Inches.of(MIDDLE_BLUE_BARGE_Y), Rotation2d.kZero),
        new Pose2d(Inches.of(BLUE_BARGE_X), Inches.of(RIGHT_BLUE_BARGE_Y), Rotation2d.kZero),

        new Pose2d(Inches.of(RED_BARGE_X), Inches.of(LEFT_RED_BARGE_Y), Rotation2d.kZero),
        new Pose2d(Inches.of(RED_BARGE_X), Inches.of(MIDDLE_RED_BARGE_Y), Rotation2d.kZero),
        new Pose2d(Inches.of(RED_BARGE_X), Inches.of(RIGHT_RED_BARGE_Y), Rotation2d.kZero),};
  };

  public static Pose2d[] makeAutoPositions(Distance autoOffset) {
    Transform2d REEF_BRANCH_TO_ROBOT = new Transform2d(
        Inches.of(-INCHES_FROM_REEF).minus(autoOffset), Inches.zero(), Rotation2d.kZero);
    return new Pose2d[] {
        new Pose2d(
            BLUE_REEF_CENTER.plus(new Translation2d(Inches.of(-4.755423), Inches.of(-21.200635))),
            Rotation2d.fromDegrees(60)).transformBy(REEF_BRANCH_TO_ROBOT),
        new Pose2d(
            BLUE_REEF_CENTER.plus(new Translation2d(Inches.of(-4.755423), Inches.of(21.200635))),
            Rotation2d.fromDegrees(300)).transformBy(REEF_BRANCH_TO_ROBOT),

        new Pose2d(
            RED_REEF_CENTER.plus(new Translation2d(Inches.of(4.755423), Inches.of(-21.200635))),
            Rotation2d.fromDegrees(120)).transformBy(REEF_BRANCH_TO_ROBOT),
        new Pose2d(
            RED_REEF_CENTER.plus(new Translation2d(Inches.of(4.755423), Inches.of(21.200635))),
            Rotation2d.fromDegrees(240)).transformBy(REEF_BRANCH_TO_ROBOT),};
  }

  private static final Pose2d[] OUTER_REEF_POSITIONS = makeReefPositions(Inches.of(12));
  private static final Pose2d[] INNER_REEF_POSITIONS = makeReefPositions(Inches.of(0));
  private static final Pose2d[] OUTER_ALGAE_POSITIONS = makeAlgaePositions(Inches.of(15));
  private static final Pose2d[] INNER_ALGAE_POSITIONS = makeAlgaePositions(Inches.of(0));
  private static final Pose2d[] BARGE_POSITIONS = makeBargePositions(Inches.of(0));
  private static final Pose2d[] AUTO_POSITIONS = makeAutoPositions(Inches.of(0));
  private static final Pose2d[] AUTO_POSITIONS_12 = makeAutoPositions(Inches.of(12));
  private static final Pose2d[] SOURCE_POSITIONS = makeSourcePositions();
  private static final Pose2d[] LL_REEF_POSITIONS_12 = makeLLReefPositions(Inches.of(12));
  private static final Pose2d[] LL_REEF_POSITIONS = makeLLReefPositions(Inches.of(0));
  private static final Pose2d[] LEFT_REEF_POSITIONS = makeLeftReefPositions(Inches.of(0));
  private static final Pose2d[] LEFT_REEF_POSITION_12 = makeLeftReefPositions(Inches.of(12));
  private static final Pose2d[] RIGHT_REEF_POSITIONS = makeRightReefPositions(Inches.of(0));
  private static final Pose2d[] RIGHT_REEF_POSITION_12 = makeRightReefPositions(Inches.of(12));

  private static final LoggedTunableNumber ANGLE_KP =
      new LoggedTunableNumber("Tuning/SnapToPosition/Angle_kP", 7.0);
  private static final LoggedTunableNumber ANGLE_KI =
      new LoggedTunableNumber("Tuning/SnapToPosition/Angle_kI", 0.0);
  private static final LoggedTunableNumber ANGLE_KD =
      new LoggedTunableNumber("Tuning/SnapToPosition/Angle_kD", 0.4);

  private static final LoggedTunableNumber POSITION_KP =
      new LoggedTunableNumber("Tuning/SnapToPosition/Position_kP", 4);
  private static final LoggedTunableNumber POSITION_KI =
      new LoggedTunableNumber("Tuning/SnapToPosition/Position_kI", 0); // 1
  private static final LoggedTunableNumber POSITION_KD =
      new LoggedTunableNumber("Tuning/SnapToPosition/Position_kD", 0); // 1

  // Create X Position PID controller
  private static final ProfiledPIDController xController = new ProfiledPIDController(0, 0, 0,
      new TrapezoidProfile.Constraints(POSITION_MAX_VELOCITY, POSITION_MAX_ACCELERATION));

  // Create Y Position PID controller
  private static final ProfiledPIDController yController = new ProfiledPIDController(0, 0, 0,
      new TrapezoidProfile.Constraints(POSITION_MAX_VELOCITY, POSITION_MAX_ACCELERATION));

  // Create Angle PID controller
  private static final ProfiledPIDController angleController = new ProfiledPIDController(0, 0, 0,
      new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));

  static {
    // Setup PID controllers
    xController.setTolerance(POSITION_TOLERANCE);
    yController.setTolerance(POSITION_TOLERANCE);
    angleController.enableContinuousInput(-Math.PI, Math.PI);
    angleController.setTolerance(ANGLE_TOLERANCE);

    // This also sets the PID gains immediately
    POSITION_KP.addListener(xController::setP);
    POSITION_KI.addListener(xController::setI);
    POSITION_KD.addListener(xController::setD);
    POSITION_KP.addListener(yController::setP);
    POSITION_KI.addListener(yController::setI);
    POSITION_KD.addListener(yController::setD);
    ANGLE_KP.addListener(angleController::setP);
    ANGLE_KI.addListener(angleController::setI);
    ANGLE_KD.addListener(angleController::setD);
  }

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

    return Commands.run(() -> {
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

        // Reset PID controller command starts
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()))
        .withName("DriveCommands.joyStickDrive`"); // when
  }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   */
  public static Command joystickDriveAtAngle(Drive drive, DoubleSupplier xSupplier,
      DoubleSupplier ySupplier, Supplier<Rotation2d> rotationSupplier) {

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
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()))
        .withName("DriveCommands.joystickDriveAtAngle");
  }

  public static Command snapToRotation(Drive drive, Rotation2d rotation) {
    return Commands.runOnce(() -> {
      Logger.recordOutput("Rotation", "snap to rotation");
      drive.setDesiredRotation(rotation);
    }).withName("DriveCommands.snapToRotation");
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
    }).withName("DriveCommands.keepRotationForward");
  }

  public static Command Snapper(Drive drive) {

    return Commands.defer(() -> {
      Pose2dSequence desiredPose =
          getClosestReefPosition(drive, Meters.of(1000)).orElse(Pose2dSequence.kZero);
      Logger.recordOutput("SnapperPose", desiredPose.outer);
      return snapToPosition(drive, desiredPose.outer)
          .andThen(snapToPosition(drive, desiredPose.inner));
    }, Set.of(drive)).withName("DriveCommands.Snapper");

  }

  public static Command LeftSnapper(Drive drive) {

    return Commands.defer(() -> {
      Pose2dSequence desiredPose =
          getClosestLeftPosition(drive, Meters.of(1000)).orElse(Pose2dSequence.kZero);
      Logger.recordOutput("SnapperPose", desiredPose.outer);
      return snapToPosition(drive, desiredPose.outer)
          .andThen(snapToPosition(drive, desiredPose.inner));
    }, Set.of(drive)).withName("DriveCommands.Snapper");

  }

  public static Command RightSnapper(Drive drive) {

    return Commands.defer(() -> {
      Pose2dSequence desiredPose =
          getClosestRightPosition(drive, Meters.of(1000)).orElse(Pose2dSequence.kZero);
      Logger.recordOutput("SnapperPose", desiredPose.outer);
      return snapToPosition(drive, desiredPose.outer)
          .andThen(snapToPosition(drive, desiredPose.inner));
    }, Set.of(drive)).withName("DriveCommands.Snapper");

  }


  public static Command AlgaeSnapper(Drive drive) {

    return Commands.defer(() -> {
      Pose2dSequence desiredPose =
          getClosestAlgaePosition(drive, Meters.of(1000)).orElse(Pose2dSequence.kZero);
      Logger.recordOutput("AlgaeSnapperPose", desiredPose.outer);
      return snapToPosition(drive, desiredPose.outer)
          .andThen(snapToPosition(drive, desiredPose.inner));
    }, Set.of(drive)).withName("DriveCommands.AlgaeSnapper");

  }

  public static Command SourceSnapper(Drive drive) {

    return Commands.defer(() -> {
      Pose2d desiredPose = getClosestSource(drive, Meters.of(1000)).orElse(Pose2d.kZero);
      Logger.recordOutput("SourceSnapperPose", desiredPose);
      return snapToPosition(drive, desiredPose);
    }, Set.of(drive)).withName("DriveCommands.SourceSnapper");

  }


  public static Command BargeSnapper(Drive drive) {

    return Commands.defer(() -> {
      Pose2d desiredPose = getClosestBargePosition(drive, Meters.of(1000)).orElse(Pose2d.kZero);
      Logger.recordOutput("BargeSnapperPose", desiredPose);
      return snapToPosition(drive, desiredPose);
    }, Set.of(drive)).withName("DriveCommands.BargeSnapper");

  }

  public static Command FirstSnapper(Drive drive) {

    return Commands.defer(() -> {
      Pose2dSequence desiredPose =
          getClosestAuto(drive, Meters.of(1000)).orElse(Pose2dSequence.kZero);
      Logger.recordOutput("SnapperPose", desiredPose.outer);
      return snapToPosition(drive, desiredPose.inner);
    }, Set.of(drive)).withName("DriveCommands.FirstSnapper");

  }

  public static Command AutoSnapper(Drive drive) {

    return Commands.defer(() -> {
      Pose2dSequence desiredPose =
          getClosestReefPosition(drive, Meters.of(1000)).orElse(Pose2dSequence.kZero);
      Logger.recordOutput("SnapperPose", desiredPose.outer);
      return snapToPosition(drive, desiredPose.inner);
    }, Set.of(drive)).withName("DriveCommands.AutoSnapper");

  }

  public static Command AutoSourceRight(Drive drive) {

    return Commands.defer(() -> {
      return snapToPosition(drive,
          new Pose2d(new Translation2d(Inches.of(Right_Loading_Station_X + 5),
              Inches.of(Right_Loading_Station_Y + 5)), Rotation2d.fromDegrees(55)));
    }, Set.of(drive)).withName("DriveCommands.AutoSourceRight");

  }

  public static Command AutoSourceLeft(Drive drive) {

    return Commands.defer(() -> {
      return snapToPosition(drive,
          new Pose2d(new Translation2d(Inches.of(Left_Loading_Station_X + 5),
              Inches.of(Left_Loading_Station_Y - 5)), Rotation2d.fromDegrees(55)));
    }, Set.of(drive)).withName("DriveCommands.AutoSourceLeft");

  }

  public static Command FullSnapperOuter(Drive drive) {

    Set<Pose2d> visited = new HashSet<>(3);

    return Commands.defer(() -> {
      Pose2d desiredPose =
          getClosestFullOuter(drive, Meters.of(1000), visited).orElse(Pose2d.kZero);

      if (DriverStation.isAutonomous() && visited.contains(desiredPose)) {
        visited.add(desiredPose);
      }

      Logger.recordOutput("SnapperPose", desiredPose);
      return snapToPosition(drive, desiredPose);
    }, Set.of(drive)).withName("DriveCommands.SourceSnapper");

  }

  public static Command FullSnapperOuterAuto(Drive drive) {

    return Commands.defer(() -> {
      Pose2d desiredPose = getClosestFullOuterAuto(drive, Meters.of(1000)).orElse(Pose2d.kZero);
      Logger.recordOutput("SnapperPose", desiredPose);
      return snapToPosition(drive, desiredPose);
    }, Set.of(drive)).withName("DriveCommands.SourceSnapper");

  }


  public static Command FullSnapperInner(Drive drive) {

    return Commands.defer(() -> {
      Pose2d desiredPose = getClosestFullInner(drive, Meters.of(1000)).orElse(Pose2d.kZero);
      Logger.recordOutput("SnapperPose", desiredPose);
      return snapToPosition(drive, desiredPose);
    }, Set.of(drive)).withName("DriveCommands.SourceSnapper");

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

  private static Optional<Pose2dSequence> getClosestLeftPosition(Drive drive, Distance radius) {
    Optional<Pose2dSequence> desiredPose = Optional.empty();
    Distance minDistance = Meters.of(1000000);
    for (int i = 0; i < LEFT_REEF_POSITION_12.length; i++) {
      Pose2d pose = LEFT_REEF_POSITION_12[i];
      double distance = drive.getPose().getTranslation().getDistance(pose.getTranslation());
      Distance distanceMeasure = Meters.of(distance);
      if (distanceMeasure.lte(radius) && distanceMeasure.lte(minDistance)) {
        minDistance = distanceMeasure;
        desiredPose =
            Optional.of(new Pose2dSequence(LEFT_REEF_POSITIONS[i], LEFT_REEF_POSITION_12[i]));
      }
    }

    return desiredPose;
  };

  private static Optional<Pose2dSequence> getClosestRightPosition(Drive drive, Distance radius) {
    Optional<Pose2dSequence> desiredPose = Optional.empty();
    Distance minDistance = Meters.of(1000000);
    for (int i = 0; i < RIGHT_REEF_POSITION_12.length; i++) {
      Pose2d pose = RIGHT_REEF_POSITION_12[i];
      double distance = drive.getPose().getTranslation().getDistance(pose.getTranslation());
      Distance distanceMeasure = Meters.of(distance);
      if (distanceMeasure.lte(radius) && distanceMeasure.lte(minDistance)) {
        minDistance = distanceMeasure;
        desiredPose =
            Optional.of(new Pose2dSequence(RIGHT_REEF_POSITIONS[i], RIGHT_REEF_POSITION_12[i]));
      }
    }

    return desiredPose;
  };

  private static Optional<Pose2dSequence> getClosestAlgaePosition(Drive drive, Distance radius) {
    Optional<Pose2dSequence> desiredPose = Optional.empty();
    Distance minDistance = Meters.of(1000000);
    for (int i = 0; i < OUTER_ALGAE_POSITIONS.length; i++) {
      Pose2d pose = OUTER_ALGAE_POSITIONS[i];
      double distance = drive.getPose().getTranslation().getDistance(pose.getTranslation());
      Distance distanceMeasure = Meters.of(distance);
      if (distanceMeasure.lte(radius) && distanceMeasure.lte(minDistance)) {
        minDistance = distanceMeasure;
        desiredPose =
            Optional.of(new Pose2dSequence(INNER_ALGAE_POSITIONS[i], OUTER_ALGAE_POSITIONS[i]));
      }
    }

    return desiredPose;
  };

  private static Optional<Pose2d> getClosestSource(Drive drive, Distance radius) {
    Optional<Pose2d> desiredPose = Optional.empty();
    Distance minDistance = Meters.of(1000000);
    for (Pose2d pose : SOURCE_POSITIONS) {
      double distance = drive.getPose().getTranslation().getDistance(pose.getTranslation());
      Distance distanceMeasure = Meters.of(distance);
      if (distanceMeasure.lte(radius) && distanceMeasure.lte(minDistance)) {
        minDistance = distanceMeasure;
        desiredPose = Optional.of(pose);
      }
    }

    return desiredPose;
  };

  private static Optional<Pose2d> getClosestBargePosition(Drive drive, Distance radius) {
    Optional<Pose2d> desiredPose = Optional.empty();
    Distance minDistance = Meters.of(1000000);
    for (Pose2d pose : BARGE_POSITIONS) {
      double distance = drive.getPose().getTranslation().getDistance(pose.getTranslation());
      Distance distanceMeasure = Meters.of(distance);
      if (distanceMeasure.lte(radius) && distanceMeasure.lte(minDistance)) {
        minDistance = distanceMeasure;
        desiredPose = Optional.of(pose);
      }
    }

    return desiredPose;
  };

  private static Optional<Pose2d> getClosestFullOuter(Drive drive, Distance radius,
      Set<Pose2d> visited) {
    Optional<Pose2d> desiredPose = Optional.empty();
    Distance minDistance = Meters.of(1000000);
    for (Pose2d pose : LL_REEF_POSITIONS_12) {

      if (DriverStation.isAutonomous() && visited.contains(pose)) {
        Logger.recordOutput("Snapper/SkippedPose", pose);
        continue;
      }

      double distance = drive.getPose().getTranslation().getDistance(pose.getTranslation());
      Distance distanceMeasure = Meters.of(distance);
      if (distanceMeasure.lte(radius) && distanceMeasure.lte(minDistance)) {
        minDistance = distanceMeasure;
        desiredPose = Optional.of(pose);
      }
    }

    return desiredPose;
  };

  private static Optional<Pose2d> getClosestFullOuterAuto(Drive drive, Distance radius) {
    Optional<Pose2d> desiredPose = Optional.empty();
    Distance minDistance = Meters.of(1000000);
    for (Pose2d pose : AUTO_POSITIONS_12) {
      double distance = drive.getPose().getTranslation().getDistance(pose.getTranslation());
      Distance distanceMeasure = Meters.of(distance);
      if (distanceMeasure.lte(radius) && distanceMeasure.lte(minDistance)) {
        minDistance = distanceMeasure;
        desiredPose = Optional.of(pose);
      }
    }

    return desiredPose;
  };

  private static Optional<Pose2d> getClosestFullInner(Drive drive, Distance radius) {
    Optional<Pose2d> desiredPose = Optional.empty();
    Distance minDistance = Meters.of(1000000);
    for (Pose2d pose : LL_REEF_POSITIONS) {
      double distance = drive.getPose().getTranslation().getDistance(pose.getTranslation());
      Distance distanceMeasure = Meters.of(distance);
      if (distanceMeasure.lte(radius) && distanceMeasure.lte(minDistance)) {
        minDistance = distanceMeasure;
        desiredPose = Optional.of(pose);
      }
    }

    return desiredPose;
  };

  private static Optional<Pose2dSequence> getClosestAuto(Drive drive, Distance radius) {
    Optional<Pose2dSequence> desiredPose = Optional.empty();
    Distance minDistance = Meters.of(1000000);
    for (int i = 0; i < AUTO_POSITIONS_12.length; i++) {
      Pose2d pose = AUTO_POSITIONS_12[i];
      double distance = drive.getPose().getTranslation().getDistance(pose.getTranslation());
      Distance distanceMeasure = Meters.of(distance);
      if (distanceMeasure.lte(radius) && distanceMeasure.lte(minDistance)) {
        minDistance = distanceMeasure;
        desiredPose = Optional.of(new Pose2dSequence(AUTO_POSITIONS[i], AUTO_POSITIONS_12[i]));
      }
    }

    return desiredPose;
  };

  // private static Optional<Pose2d> getClosestSource(Drive drive, Distance
  // radius) {
  // Optional<Pose2dSequence> desiredPose = Optional.empty();
  // Distance minDistance = Meters.of(1000000);
  // for (int i = 0; i < SOURCE_POSITIONS.length; i++) {
  // Pose2d pose = SOURCE_POSITIONS[i];
  // double distance =
  // drive.getPose().getTranslation().getDistance(pose.getTranslation());
  // Distance distanceMeasure = Meters.of(distance);
  // if (distanceMeasure.lte(radius) && distanceMeasure.lte(minDistance)) {
  // minDistance = distanceMeasure;
  // desiredPose = Optional.of(new Pose2d(SOURCE_POSITIONS[i]));
  // }
  // }

  // return desiredPose;
  // };

  public static Command snapToPosition(Drive drive, Pose2d desiredPosition) {
    return Commands.run(() -> {

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
          var fieldRelativeSpeeds = drive.getFieldRelativeSpeeds();
          angleController.reset(drive.getRotation().getRadians(),
              fieldRelativeSpeeds.omegaRadiansPerSecond);
          xController.reset(drive.getPose().getX(), fieldRelativeSpeeds.vxMetersPerSecond);
          yController.reset(drive.getPose().getY(), fieldRelativeSpeeds.vyMetersPerSecond);
        }).until(() -> angleController.atGoal() && xController.atGoal() && yController.atGoal())
        .withName("DriveCommands.snapToPosition");
  }

  public static Command joystickDriveAssist(Drive drive, DoubleSupplier xSupplier,
      DoubleSupplier ySupplier, DoubleSupplier omegaSupplier, BooleanSupplier snapSupplier,
      BooleanSupplier slowModeSupplier) {

    return Commands.run(() -> {

      Logger.recordOutput("InnerReefPositions", DriveCommands.INNER_REEF_POSITIONS);
      Logger.recordOutput("InnerAlgaePositions", DriveCommands.INNER_ALGAE_POSITIONS);
      Logger.recordOutput("OuterReefPositions", DriveCommands.OUTER_REEF_POSITIONS);
      Logger.recordOutput("OuterAlgaePositions", DriveCommands.OUTER_ALGAE_POSITIONS);
      Logger.recordOutput("BargePositions", DriveCommands.BARGE_POSITIONS);

      final double isRed =
          DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? 1 : -1;

      final double slowModeMultiplier =
          (slowModeSupplier.getAsBoolean() ? SLOW_MODE_MULTIPLIER : 1.0);

      // Get linear velocity
      Translation2d linearVelocity = getLinearVelocityFromJoysticks(
          Math.copySign(xSupplier.getAsDouble() * xSupplier.getAsDouble(), xSupplier.getAsDouble()),
          Math.copySign(ySupplier.getAsDouble() * ySupplier.getAsDouble(),
              ySupplier.getAsDouble()));

      // Apply rotation deadband
      double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

      final double maxSpeed = drive.getMaxLinearSpeedMetersPerSec();

      double linearX = linearVelocity.getX() * maxSpeed * slowModeMultiplier * isRed;
      double linearY = linearVelocity.getY() * maxSpeed * slowModeMultiplier * isRed;

      // Square rotation value for more precise control
      omega = Math.copySign(omega * omega, omega);

      omega *= drive.getMaxAngularSpeedRadPerSec();
      if ((Math.abs(omega) > 1E-6) || (Math.abs(linearX) > 1E-6) || (Math.abs(linearY) > 1E-6)) {
        Logger.recordOutput("DriveState", "Driver");
        Logger.recordOutput("Snap/desiredPos", new Pose2d(-50, -50, Rotation2d.kZero));
      } else if (snapSupplier.getAsBoolean()) {
        Optional<Pose2dSequence> closestOptionalPose =
            getClosestAlgaePosition(drive, SNAPPY_RADIUS);

        if (closestOptionalPose.isPresent()) {
          Pose2dSequence closestPoseSequence = closestOptionalPose.orElse(Pose2dSequence.kZero);
          Pose2d closestPose = closestPoseSequence.inner;
          Logger.recordOutput("DriveState", "Robot");
          Logger.recordOutput("Snap/desiredPos", closestPose);
          if (angleController.atGoal() && xController.atGoal() && yController.atGoal()) {
            linearX = 0;
            linearY = 0;
            omega = 0;
          } else {
            linearX = xController.calculate(drive.getPose().getX(), closestPose.getX());

            linearY = yController.calculate(drive.getPose().getY(), closestPose.getY());

            omega = angleController.calculate(drive.getRotation().getRadians(),
                closestPose.getRotation().getRadians());
          }

        }
      }

      Logger.recordOutput("Snap/omega", omega);
      Logger.recordOutput("Snap/x/xDiff", linearX);
      Logger.recordOutput("Snap/x/currentXPos", drive.getPose().getX());
      Logger.recordOutput("Snap/y/yDiff", linearY);
      Logger.recordOutput("Snap/y/currentYPos", drive.getPose().getY());

      // Convert to field relative speeds & send command
      ChassisSpeeds speeds = new ChassisSpeeds(linearX, linearY, omega);
      drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drive.getRotation()));

    }, drive)

        // Reset PID controller when command starts
        .beforeStarting(() -> {
          var fieldRelativeSpeeds = drive.getFieldRelativeSpeeds();
          angleController.reset(drive.getRotation().getRadians(),
              fieldRelativeSpeeds.omegaRadiansPerSecond);
          xController.reset(drive.getPose().getX(), fieldRelativeSpeeds.vxMetersPerSecond);
          yController.reset(drive.getPose().getY(), fieldRelativeSpeeds.vyMetersPerSecond);
        }).withName("DriveCommands.joystickDriveAssist");
  }

  public static Command expoAssist(Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier, BooleanSupplier snapSupplier,
      BooleanSupplier slowModeSupplier) {

    return Commands.run(() -> {

      System.out.println("Expo Assist");

      final double isRed =
          DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? 1 : -1;

      final double slowModeMultiplier =
          (slowModeSupplier.getAsBoolean() ? SLOW_MODE_MULTIPLIER : 1.0);

      // Get exponential velocity
      Translation2d exponentialVelocity = getLinearVelocityFromJoysticks(
          Math.pow(-xSupplier.getAsDouble(), (3)), Math.pow(-ySupplier.getAsDouble(), (3)));

      // Apply rotation deadband
      double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

      final double maxSpeed = drive.getMaxLinearSpeedMetersPerSec();

      double exponentialX = exponentialVelocity.getX() * maxSpeed * slowModeMultiplier * isRed;
      double exponentialY = exponentialVelocity.getY() * maxSpeed * slowModeMultiplier * isRed;

      // Square rotation value for more precise control
      omega = Math.copySign(omega * omega, omega);

      omega *= drive.getMaxAngularSpeedRadPerSec();
      if ((Math.abs(omega) > 1E-6) || (Math.abs(exponentialX) > 1E-6)
          || (Math.abs(exponentialY) > 1E-6)) {
        Logger.recordOutput("DriveState", "Driver");
        Logger.recordOutput("Snap/desiredPos", new Pose2d(-50, -50, Rotation2d.kZero));
      } else if (snapSupplier.getAsBoolean()) {
        Optional<Pose2dSequence> closestOptionalPose = getClosestReefPosition(drive, SNAPPY_RADIUS);

        if (closestOptionalPose.isPresent()) {
          Pose2dSequence closestPoseSequence = closestOptionalPose.orElse(Pose2dSequence.kZero);
          Pose2d closestPose = closestPoseSequence.inner;
          Logger.recordOutput("DriveState", "Robot");
          Logger.recordOutput("Snap/desiredPos", closestPose);
          if (angleController.atGoal() && xController.atGoal() && yController.atGoal()) {
            exponentialX = 0;
            exponentialY = 0;
            omega = 0;
          } else {
            exponentialX = xController.calculate(drive.getPose().getX(), closestPose.getX());

            exponentialY = yController.calculate(drive.getPose().getY(), closestPose.getY());

            omega = angleController.calculate(drive.getRotation().getRadians(),
                closestPose.getRotation().getRadians());
          }

        }
      }

      // Convert to field relative speeds & send command
      ChassisSpeeds speeds = new ChassisSpeeds(exponentialX, exponentialY, omega);
      drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drive.getRotation()));

    }, drive)

        // Reset PID controller when command starts
        .beforeStarting(() -> {
          var fieldRelativeSpeeds = drive.getFieldRelativeSpeeds();
          angleController.reset(drive.getRotation().getRadians(),
              fieldRelativeSpeeds.omegaRadiansPerSecond);
          xController.reset(drive.getPose().getX(), fieldRelativeSpeeds.vxMetersPerSecond);
          yController.reset(drive.getPose().getY(), fieldRelativeSpeeds.vyMetersPerSecond);
        });
  }
}
