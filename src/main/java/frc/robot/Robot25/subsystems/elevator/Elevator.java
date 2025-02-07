package frc.robot.Robot25.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Radians;
import static frc.robot.Robot25.subsystems.elevator.ElevatorConstants.DRUM_RADIUS;
import static frc.robot.Robot25.subsystems.elevator.ElevatorConstants.INITIAL_HEIGHT;
import static frc.robot.Robot25.subsystems.elevator.ElevatorConstants.MAX_EXTENSION;
import static frc.robot.Robot25.subsystems.elevator.ElevatorConstants.MIN_HEIGHT;

import com.ctre.phoenix6.wpiutils.ReplayAutoEnable;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class Elevator extends SubsystemBase {

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  @AutoLogOutput
  public final LoggedMechanism2d mechanism2d =
      new LoggedMechanism2d(3, 3, new Color8Bit(Color.kBlack));

  private final LoggedMechanismRoot2d mechRoot2d = mechanism2d.getRoot("Elevator Root", 1.5, 0);
  private final LoggedMechanismLigament2d elevatorMech2d =
      mechRoot2d.append(new LoggedMechanismLigament2d("Elevator", INITIAL_HEIGHT.in(Meters), 90.0,
          50, new Color8Bit(Color.kBlue)));

  private enum Level {

    minHeight(MIN_HEIGHT), L1(Inches.of(18 + 3)), L2(Inches.of(31.9 + 3)), L3(
        Inches.of(47.6 + 3)), L4(Inches.of(72 + 3));

    private final Distance height;

    Level(Distance height) {
      this.height = height;
    }

    public Distance getHeight() {
      return height;
    }

    public Level up() {
      switch (this) {
        case minHeight:
          return L1;
        case L1:
          return L2;
        case L2:
          return L3;
        case L3:
          return L4;
        default:
          return L4;
      }
    }

    public Level down() {
      switch (this) {
        case L4:
          return L3;
        case L3:
          return L2;
        case L2:
          return L1;
        case L1:
          return minHeight;
        default:
          return minHeight;
      }
    }

  };

  private Level currentLevel;

  public Elevator(ElevatorIO io) {
    this.io = io;
    currentLevel = Level.minHeight;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    elevatorMech2d.setLength(radiansToInches(inputs.winchPosition).in(Meters));

    Logger.recordOutput("Elevator/EstimatedHeight",
        radiansToInches(inputs.winchPosition).in(Meters));

    Logger.recordOutput("Elevator/CurrentLevel", currentLevel);
    Logger.recordOutput("Elevator/CurrentLevelHeight", currentLevel.getHeight());
    Logger.recordOutput("Elevator/Radians", inchesToRadians(currentLevel.getHeight()));

    Logger.recordOutput("Elevator/PTuning",
        currentLevel.getHeight().minus(radiansToInches(inputs.winchPosition)));
  }

  private Angle inchesToRadians(Distance d) {
    return Radians.of(d.minus(MIN_HEIGHT).in(Meters) / DRUM_RADIUS.in(Meters));
  }

  private Distance radiansToInches(Angle a) {
    double d = a.in(Radians) * DRUM_RADIUS.in(Meters);
    return Meters.of(d).plus(MIN_HEIGHT);
  }

  public Command goToLevel(Level level) {
    return this.runOnce(() -> {
      currentLevel = level;
      Distance height = level.getHeight();
      Angle r = inchesToRadians(height);
      io.setWinchPosition(r);
    }).andThen(Commands.waitUntil(() -> {
      return Math.abs(
          inputs.winchPosition.in(Radians) - inchesToRadians(level.getHeight()).in(Radians)) < 0.1;
    }));
  }

  public Command minHeight() {
    return goToLevel(Level.minHeight);
  }

  public Command L1() {
    return goToLevel(Level.L1);
  }


  public Command L2() {
    return goToLevel(Level.L2);
  }

  public Command L3() {
    return goToLevel(Level.L3);
  }

  public Command L4() {
    return goToLevel(Level.L4);
  }

  public Command maxHeight() {
    return goToLevel(Level.L4);
  }

  public Command upLevel() {
    return goToLevel(currentLevel.up());
  }

  public Command downLevel() {
    return goToLevel(currentLevel.down());
  }

  /**
   * Computes the estimated position of each elevator stage (stage 1, stage 2, carriage + loader)
   * given the current winch position. Useful for rendering the robot model in simulation.
   *
   * A continuous elevator lifts stages in order of least weight. As of 2/1, for us that would be
   * stage 2, stage 1, then carriage. In order, each stage will contribute as much height as it can
   * to the lift before the next stage engages.
   *
   * @returns Pose array of each elevator stage in the order: stage 1, stage 2, carriage
   */
  public Pose3d[] getElevatorPoses() {
    var height = radiansToInches(inputs.winchPosition).minus(MIN_HEIGHT);
    var heightInches = height.in(Inches);
    // contributes no height until above 26.25 inches and contributes up to 25.25
    // inches
    var stage1Contribution = heightInches < 26.25 ? 0
        : heightInches < 51.5 ? height.minus(Inches.of(26.25)).in(Meters)
            : Inches.of(25.25).in(Meters);
    // contributes up to 26.25 inches starting at height 0
    var stage2Contribution = heightInches < 26.25 ? height.in(Meters) : Inches.of(26.25).in(Meters);
    // contributes no height until above 51.5 inches and contributes up to 25.25
    // inches
    var stage3Contribution = heightInches < 51.5 ? 0 : height.minus(Inches.of(51.5)).in(Meters);

    return new Pose3d[] {new Pose3d(0, 0, stage1Contribution, Rotation3d.kZero),
        new Pose3d(0, 0, stage1Contribution + stage2Contribution, Rotation3d.kZero), new Pose3d(0,
            0, stage1Contribution + stage2Contribution + stage3Contribution, Rotation3d.kZero),};
  }

  public Trigger elevatorAtMinHeight() {
    return new Trigger(() -> currentLevel == Level.minHeight);
  }
}
