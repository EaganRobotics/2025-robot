package frc.robot.Robot25.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Radians;
import static frc.robot.Robot25.subsystems.elevator.ElevatorConstants.DRUM_RADIUS;
import static frc.robot.Robot25.subsystems.elevator.ElevatorConstants.INITIAL_HEIGHT;
import static frc.robot.Robot25.subsystems.elevator.ElevatorConstants.MAX_EXTENSION;
import static frc.robot.Robot25.subsystems.elevator.ElevatorConstants.MIN_HEIGHT;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  @AutoLogOutput
  public final LoggedMechanism2d mechanism2d = new LoggedMechanism2d(3, 3, new Color8Bit(Color.kBlack));

  private final LoggedMechanismRoot2d mechRoot2d = mechanism2d.getRoot("Elevator Root", 1.5, 0);
  private final LoggedMechanismLigament2d elevatorMech2d = mechRoot2d
      .append(new LoggedMechanismLigament2d("Elevator", INITIAL_HEIGHT.in(Meters), 90.0,
          50, new Color8Bit(Color.kBlue)));

  public Elevator(ElevatorIO io) {
    this.io = io;
    this.minHeight();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    elevatorMech2d.setLength(radiansToInches(inputs.winchPosition).in(Meters));

    Logger.recordOutput("Elevator/EstimatedHeight",
        radiansToInches(inputs.winchPosition).in(Meters));
  }

  private Angle inchesToRadians(Distance d) {
    d = d.minus(MIN_HEIGHT);
    return Radians.of(d.in(Meters) / DRUM_RADIUS.in(Meters));
  }

  private Distance radiansToInches(Angle a) {
    double d = a.in(Radians) * DRUM_RADIUS.in(Meters);
    return Meters.of(d).plus(MIN_HEIGHT);
  }

  public void minHeight() {
    Distance height = MIN_HEIGHT;
    Angle r = inchesToRadians(height);
    io.setWinchPosition(r);
  }

  public void L1() {
    Distance height = Inches.of(18 + 3);
    Angle r = inchesToRadians(height);
    io.setWinchPosition(r);
  }

  public void L2() {
    Distance height = Inches.of(31.9 + 3);
    Angle r = inchesToRadians(height);
    io.setWinchPosition(r);
  }

  public void L3() {
    Distance height = Inches.of(47.6 + 3);
    Angle r = inchesToRadians(height);
    io.setWinchPosition(r);
  }

  public void L4() {
    Distance height = Inches.of(72 + 3);
    Angle r = inchesToRadians(height);
    io.setWinchPosition(r);
  }

  public void maxHeight() {
    Distance height = MAX_EXTENSION.plus(MIN_HEIGHT);
    Angle r = inchesToRadians(height);
    io.setWinchPosition(r);
  }

  public Pose3d[] getElevatorPoses() {
    var height = radiansToInches(inputs.winchPosition).minus(MIN_HEIGHT);
    var stageLift = height.in(Meters) / 3;
    // cascading lift: first stage is 1/3 current height, second stage is 2/3
    // current height, and third stage is at lift height
    return new Pose3d[] { new Pose3d(0, 0, stageLift, Rotation3d.kZero),
        new Pose3d(0, 0, 2 * stageLift, Rotation3d.kZero), new Pose3d(0, 0, stageLift * 3, Rotation3d.kZero) };
  }
}
