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

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.RobotContainer;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Optional;
import java.util.function.Supplier;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class SimConstants {

  /* Simulation mode, initial position, and robot container */

  /*
   * This gets the value of environment variable AKIT_SIM_MODE otherwise sets SIM_MODE to Mode.SIM
   */
  public static final Mode SIM_MODE =
      Optional.ofNullable(System.getenv("AKIT_SIM_MODE")).map(Mode::valueOf).orElse(Mode.SIM);

  public static final Pose2d SIM_INITIAL_FIELD_POSE = new Pose2d(3.28, 3.86, new Rotation2d());
  public static final Supplier<RobotContainer> SIM_ROBOT_SUPPLIER =
      () -> new frc.robot.Robot25.RobotContainer();

  public static final Pose3d HIDDEN_CORAL_POSE =
      new Pose3d(-100, -100, -100, new Rotation3d(0, 0, 0));
  public static final Pose3d LEFT_STATION_CORAL_POSE = new Pose3d(0.636, 7.693, 1.265,
      new Rotation3d(Degrees.zero(), Degrees.of(35), Degrees.of(-54)));
  public static final Pose3d RIGHT_STATION_CORAL_POSE = new Pose3d(0.641, 0.359, 1.280,
      new Rotation3d(Degrees.zero(), Degrees.of(35), Degrees.of(54)));
  public static final Distance LOADING_STATION_WIDTH = Meters.of(1.626);

  public static final Translation2d FR_LOADING_STATION = new Translation2d(1.66, 0.52);
  public static final Translation2d BR_LOADING_STATION = new Translation2d(0.53, 1.38);
  public static final Translation2d FL_LOADING_STATION = new Translation2d(1.66, 7.53);
  public static final Translation2d BL_LOADING_STATION = new Translation2d(0.53, 6.71);
  public static final Distance LOADING_STATION_TOLERANCE = Inches.of(2);

  public static final Time LOAD_CORAL_DELAY = Seconds.of(1.5);

  /* DO NOT CHANGE */
  public static final Mode CURRENT_MODE = RobotBase.isReal() ? Mode.REAL : SIM_MODE;

  public enum Mode {
    /** Running on a real robot. */
    REAL(null),

    /** Running a physics simulator. */
    SIM("SIM"),

    /** Replaying from a log file. */
    REPLAY("REPLAY");

    Mode(String env) {}
  }
}
