package frc.lib;

import static edu.wpi.first.units.Units.Seconds;

import java.util.function.DoubleSupplier;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class MockXboxController extends XboxControllerSim {

  private static final Time DEFAULT_PRESS_DURATION_SEC = Seconds.of(0.1);

  public static final MockXboxController DRIVER_CONTROLLER = new MockXboxController(0);
  public static final MockXboxController OPERATOR_CONTROLLER = new MockXboxController(1);

  public MockXboxController(int port) {
    super(port);
    DriverStationSim.setJoystickIsXbox(port, true);
  }

  public Command pressAButton(Time time) {
    return pressButton(1, time);
  }

  public Command pressAButton(double seconds) {
    return pressButton(1, Seconds.of(seconds));
  }

  public Command pressAButton() {
    return pressButton(1, DEFAULT_PRESS_DURATION_SEC);
  }

  public Command pressBButton(Time time) {
    return pressButton(2, time);
  }

  public Command pressBButton(double seconds) {
    return pressButton(2, Seconds.of(seconds));
  }

  public Command pressBButton() {
    return pressButton(2, DEFAULT_PRESS_DURATION_SEC);
  }

  public Command pressXButton(Time time) {
    return pressButton(3, time);
  }

  public Command pressXButton(double seconds) {
    return pressButton(3, Seconds.of(seconds));
  }

  public Command pressXButton() {
    return pressButton(3, DEFAULT_PRESS_DURATION_SEC);
  }

  public Command pressYButton(Time time) {
    return pressButton(4, time);
  }

  public Command pressYButton(double seconds) {
    return pressButton(4, Seconds.of(seconds));
  }

  public Command pressYButton() {
    return pressButton(4, DEFAULT_PRESS_DURATION_SEC);
  }

  public Command pressLeftBumper(Time time) {
    return pressButton(5, time);
  }

  public Command pressLeftBumper(double seconds) {
    return pressButton(5, Seconds.of(seconds));
  }

  public Command pressLeftBumper() {
    return pressButton(5, DEFAULT_PRESS_DURATION_SEC);
  }

  public Command pressRightBumper(Time time) {
    return pressButton(6, time);
  }

  public Command pressRightBumper(double seconds) {
    return pressButton(6, Seconds.of(seconds));
  }

  public Command pressRightBumper() {
    return pressButton(6, DEFAULT_PRESS_DURATION_SEC);
  }

  public Command pressBackButton(Time time) {
    return pressButton(7, time);
  }

  public Command pressBackButton(double seconds) {
    return pressButton(7, Seconds.of(seconds));
  }

  public Command pressBackButton() {
    return pressButton(7, DEFAULT_PRESS_DURATION_SEC);
  }

  public Command pressStartButton(Time time) {
    return pressButton(8, time);
  }

  public Command pressStartButton(double seconds) {
    return pressButton(8, Seconds.of(seconds));
  }

  public Command pressStartButton() {
    return pressButton(8, DEFAULT_PRESS_DURATION_SEC);
  }

  public Command pressLeftStickButton(Time time) {
    return pressButton(9, time);
  }

  public Command pressLeftStickButton(double seconds) {
    return pressButton(9, Seconds.of(seconds));
  }

  public Command pressLeftStickButton() {
    return pressButton(9, DEFAULT_PRESS_DURATION_SEC);
  }

  public Command pressRightStickButton(Time time) {
    return pressButton(10, time);
  }

  public Command pressRightStickButton(double seconds) {
    return pressButton(10, Seconds.of(seconds));
  }

  public Command pressRightStickButton() {
    return pressButton(10, DEFAULT_PRESS_DURATION_SEC);
  }

  public Command pressPovUp(Time time) {
    return pressPov(0, time);
  }

  public Command pressPovUp(double seconds) {
    return pressPov(0, Seconds.of(seconds));
  }

  public Command pressPovUp() {
    return pressPov(0, DEFAULT_PRESS_DURATION_SEC);
  }

  public Command pressPovUpRight(Time time) {
    return pressPov(45, time);
  }

  public Command pressPovUpRight(double seconds) {
    return pressPov(45, Seconds.of(seconds));
  }

  public Command pressPovUpRight() {
    return pressPov(45, DEFAULT_PRESS_DURATION_SEC);
  }

  public Command pressPovRight(Time time) {
    return pressPov(90, time);
  }

  public Command pressPovRight(double seconds) {
    return pressPov(90, Seconds.of(seconds));
  }

  public Command pressPovRight() {
    return pressPov(90, DEFAULT_PRESS_DURATION_SEC);
  }

  public Command pressPovDownRight(Time time) {
    return pressPov(135, time);
  }

  public Command pressPovDownRight(double seconds) {
    return pressPov(135, Seconds.of(seconds));
  }

  public Command pressPovDownRight() {
    return pressPov(135, DEFAULT_PRESS_DURATION_SEC);
  }

  public Command pressPovDown(Time time) {
    return pressPov(180, time);
  }

  public Command pressPovDown(double seconds) {
    return pressPov(180, Seconds.of(seconds));
  }

  public Command pressPovDown() {
    return pressPov(180, DEFAULT_PRESS_DURATION_SEC);
  }

  public Command pressPovDownLeft(Time time) {
    return pressPov(225, time);
  }

  public Command pressPovDownLeft(double seconds) {
    return pressPov(225, Seconds.of(seconds));
  }

  public Command pressPovDownLeft() {
    return pressPov(225, DEFAULT_PRESS_DURATION_SEC);
  }

  public Command pressPovLeft(Time time) {
    return pressPov(270, time);
  }

  public Command pressPovLeft(double seconds) {
    return pressPov(270, Seconds.of(seconds));
  }

  public Command pressPovLeft() {
    return pressPov(270, DEFAULT_PRESS_DURATION_SEC);
  }

  public Command pressPovUpLeft(Time time) {
    return pressPov(315, time);
  }

  public Command pressPovUpLeft(double seconds) {
    return pressPov(315, Seconds.of(seconds));
  }

  public Command pressPovUpLeft() {
    return pressPov(315, DEFAULT_PRESS_DURATION_SEC);
  }

  public Command pressPov(int pov, Time time) {
    return Commands.startEnd(() -> {
      setPOV(0, pov);
      this.notifyNewData();
    }, () -> {
      setPOV(0, -1);
      this.notifyNewData();
    })
        .withTimeout(time);
  }

  public Command pressPov(int pov, double seconds) {
    return pressPov(pov, Seconds.of(seconds));
  }

  public Command pressPov(int pov) {
    return pressPov(pov, DEFAULT_PRESS_DURATION_SEC);
  }

  public Command holdLeftX(double value, Time time) {
    return holdAxis(0, value, time);
  }

  public Command holdLeftX(double value, double seconds) {
    return holdAxis(0, value, Seconds.of(seconds));
  }

  public Command moveLeftX(DoubleSupplier valueSupplier, Time time) {
    return moveAxis(0, valueSupplier, time);
  }

  public Command moveLeftX(DoubleSupplier valueSupplier, double seconds) {
    return moveAxis(0, valueSupplier, Seconds.of(seconds));
  }

  public Command lerpLeftX(Time time, double... values) {
    return lerpAxis(0, time, values);
  }

  public Command lerpLeftX(double seconds, double... values) {
    return lerpAxis(0, Seconds.of(seconds), values);
  }

  public Command holdLeftY(double value, Time time) {
    return holdAxis(1, value, time);
  }

  public Command holdLeftY(double value, double seconds) {
    return holdAxis(1, value, Seconds.of(seconds));
  }

  public Command moveLeftY(DoubleSupplier valueSupplier, Time time) {
    return moveAxis(1, valueSupplier, time);
  }

  public Command moveLeftY(DoubleSupplier valueSupplier, double seconds) {
    return moveAxis(1, valueSupplier, Seconds.of(seconds));
  }

  public Command lerpLeftY(Time time, double... values) {
    return lerpAxis(1, time, values);
  }

  public Command lerpLeftY(double seconds, double... values) {
    return lerpAxis(1, Seconds.of(seconds), values);
  }

  public Command holdLeftTrigger(double value, Time time) {
    return holdAxis(2, value, time);
  }

  public Command holdLeftTrigger(double value, double seconds) {
    return holdAxis(2, value, Seconds.of(seconds));
  }

  public Command moveLeftTrigger(DoubleSupplier valueSupplier, Time time) {
    return moveAxis(2, valueSupplier, time);
  }

  public Command moveLeftTrigger(DoubleSupplier valueSupplier, double seconds) {
    return moveAxis(2, valueSupplier, Seconds.of(seconds));
  }

  public Command lerpLeftTrigger(Time time, double... values) {
    return lerpAxis(2, time, values);
  }

  public Command lerpLeftTrigger(double seconds, double... values) {
    return lerpAxis(2, Seconds.of(seconds), values);
  }

  public Command holdRightTrigger(double value, Time time) {
    return holdAxis(3, value, time);
  }

  public Command holdRightTrigger(double value, double seconds) {
    return holdAxis(3, value, Seconds.of(seconds));
  }

  public Command moveRightTrigger(DoubleSupplier valueSupplier, Time time) {
    return moveAxis(3, valueSupplier, time);
  }

  public Command moveRightTrigger(DoubleSupplier valueSupplier, double seconds) {
    return moveAxis(3, valueSupplier, Seconds.of(seconds));
  }

  public Command lerpRightTrigger(Time time, double... values) {
    return lerpAxis(3, time, values);
  }

  public Command lerpRightTrigger(double seconds, double... values) {
    return lerpAxis(3, Seconds.of(seconds), values);
  }

  public Command holdRightX(double value, Time time) {
    return holdAxis(4, value, time);
  }

  public Command holdRightX(double value, double seconds) {
    return holdAxis(4, value, Seconds.of(seconds));
  }

  public Command moveRightX(double value, Time time) {
    return moveAxis(4, () -> value, time);
  }

  public Command moveRightX(double value, double seconds) {
    return moveAxis(4, () -> value, Seconds.of(seconds));
  }

  public Command lerpRightX(Time time, double... values) {
    return lerpAxis(4, time, values);
  }

  public Command lerpRightX(double seconds, double... values) {
    return lerpAxis(4, Seconds.of(seconds), values);
  }

  public Command holdRightY(double value, Time time) {
    return holdAxis(5, value, time);
  }

  public Command holdRightY(double value, double seconds) {
    return holdAxis(5, value, Seconds.of(seconds));
  }

  public Command moveRightY(double value, Time time) {
    return moveAxis(5, () -> value, time);
  }

  public Command moveRightY(double value, double seconds) {
    return moveAxis(5, () -> value, Seconds.of(seconds));
  }

  public Command lerpRightY(Time time, double... values) {
    return lerpAxis(5, time, values);
  }

  public Command lerpRightY(double seconds, double... values) {
    return lerpAxis(5, Seconds.of(seconds), values);
  }

  private Command pressButton(int button, Time time) {
    return Commands.runEnd(() -> {
      setRawButton(button, true);
      this.notifyNewData();
    }, () -> {
      setRawButton(button, false);
      this.notifyNewData();
    })
        .withTimeout(time);
  }

  private Command holdAxis(int axis, double value, Time time) {
    return Commands.runEnd(() -> {
      setRawAxis(axis, value);
      this.notifyNewData();
    }, () -> {
      setRawAxis(axis, 0);
      this.notifyNewData();
    })
        .withTimeout(time);
  }

  private Command moveAxis(int axis, DoubleSupplier valueSupplier, Time time) {
    return Commands.runEnd(() -> {
      setRawAxis(axis, valueSupplier.getAsDouble());
      this.notifyNewData();
    }, () -> {
      setRawAxis(axis, 0);
      this.notifyNewData();
    })
        .withTimeout(time);
  }

  private Command lerpAxis(int axis, Time time, double... values) {
    if (values.length == 0) {
      return Commands.none();
    }

    double totalTimeSeconds = time.in(Seconds);
    double stepTime = totalTimeSeconds / (values.length - 1);
    double startTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

    return Commands.runEnd(() -> {
      double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
      double elapsedTime = currentTime - startTime;

      if (elapsedTime < totalTimeSeconds) {
        // Calculate which segment we're in
        int segment = Math.min((int) (elapsedTime / stepTime), values.length - 2);
        double segmentProgress = (elapsedTime - segment * stepTime) / stepTime;

        // Linear interpolation between current segment values
        double interpolatedValue = values[segment] + segmentProgress * (values[segment + 1] - values[segment]);

        setRawAxis(axis, interpolatedValue);
        this.notifyNewData();
      } else {
        // Set to final value when done
        setRawAxis(axis, values[values.length - 1]);
        this.notifyNewData();
      }
    }, () -> {
      setRawAxis(axis, 0);
      this.notifyNewData();
    })
        .withTimeout(time);
  }
}
