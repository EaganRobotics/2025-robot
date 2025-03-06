package frc.robot.Robot25.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;

/**
 *
 * Design Calculations:
 * https://www.reca.lc/linear?angle=%7B%22s%22%3A90%2C%22u%22%3A%22deg%22%7D&currentLimit=%7B%22s%22%3A40%2C%22u%22%3A%22A%22%7D&efficiency=85&limitAcceleration=0&limitDeceleration=0&limitVelocity=0&limitedAcceleration=%7B%22s%22%3A400%2C%22u%22%3A%22in%2Fs2%22%7D&limitedDeceleration=%7B%22s%22%3A50%2C%22u%22%3A%22in%2Fs2%22%7D&limitedVelocity=%7B%22s%22%3A10%2C%22u%22%3A%22in%2Fs%22%7D&load=%7B%22s%22%3A15%2C%22u%22%3A%22lbs%22%7D&motor=%7B%22quantity%22%3A2%2C%22name%22%3A%22Kraken%20X60%2A%22%7D&ratio=%7B%22magnitude%22%3A5%2C%22ratioType%22%3A%22Reduction%22%7D&spoolDiameter=%7B%22s%22%3A3%2C%22u%22%3A%22in%22%7D&travelDistance=%7B%22s%22%3A75%2C%22u%22%3A%22in%22%7D
 */
public class ElevatorConstants {

  public static final Distance DRUM_RADIUS = Inches.of(1);
  public static final double GEARING = 5.0;
  public static final Distance MIN_HEIGHT = Inches.of(16.4);
  public static final Distance INITIAL_HEIGHT = Inches.of(16.4);
  public static final Distance MAX_EXTENSION = Inches.of(80);
  public static final Mass CARRIAGE_MASS = Pounds.of(19.147);
  public static final LinearVelocity MAX_VELOCITY = InchesPerSecond.of(178.63);
  public static final AngularAcceleration MAX_ACCELERATION = RadiansPerSecondPerSecond.of(270 / DRUM_RADIUS.in(Inches));
  public static final Current CURRENT_LIMIT = Amps.of(30);

  public static final class Real {
    public static final LoggedNetworkNumber kP = new LoggedNetworkNumber("Tuning/Elevator/kP", 2.0);
    public static final LoggedNetworkNumber kI = new LoggedNetworkNumber("Tuning/Elevator/kI", 0.0);
    public static final LoggedNetworkNumber kD = new LoggedNetworkNumber("Tuning/Elevator/kD", 0.1);
    public static final LoggedNetworkNumber kS = new LoggedNetworkNumber("Tuning/Elevator/kS", 0.0);
    public static final LoggedNetworkNumber kG = new LoggedNetworkNumber("Tuning/Elevator/kG", 0.0);
    public static final LoggedNetworkNumber kV = new LoggedNetworkNumber("Tuning/Elevator/kV", 0.0);
    public static final LoggedNetworkNumber kA = new LoggedNetworkNumber("Tuning/Elevator/kA", 0.0);

    public static final double ODOMETRY_FREQUENCY = new CANBus().isNetworkFD() ? 250.0 : 100.0;
  }

  public static final class Sim {
    public static final double kP = 3.5; // 4
    public static final double kI = 1; // 0.3
    public static final double kD = 0.06; // 0.6
    public static final double kS = 0.0;
    public static final double kG = 0.548; // 0.54
    public static final double kV = 0.10146; // 0.10146
    public static final double kA = 0.002; // 0.002
    public static final MomentOfInertia MOTOR_LOAD_MOI = KilogramSquareMeters.of(0.01261); // TODO
                                                                                           // estimate
    public static final Voltage FRICTION_VOLTAGE = Volts.of(1.1);
  }
}
