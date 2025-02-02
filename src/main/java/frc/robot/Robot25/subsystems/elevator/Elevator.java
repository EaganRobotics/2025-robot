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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class Elevator extends SubsystemBase {

    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs =
        new ElevatorIOInputsAutoLogged();

    @AutoLogOutput
    public final LoggedMechanism2d mechanism2d = new LoggedMechanism2d(
        3,
        3,
        new Color8Bit(Color.kBlack)
    );

    private final LoggedMechanismRoot2d mechRoot2d = mechanism2d.getRoot(
        "Elevator Root",
        1.5,
        0
    );
    private final LoggedMechanismLigament2d elevatorMech2d = mechRoot2d.append(
        new LoggedMechanismLigament2d(
            "Elevator",
            INITIAL_HEIGHT.in(Meters),
            90.0,
            50,
            new Color8Bit(Color.kBlue)
        )
    );

    public Elevator(ElevatorIO io) {
        this.io = io;

        // TODO this doesn't work, you're calling a function that returns a command; but it does not the command
        this.minHeight();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
        elevatorMech2d.setLength(
            radiansToInches(inputs.winchPosition).in(Meters)
        );

        Logger.recordOutput(
            "Elevator/EstimatedHeight",
            radiansToInches(inputs.winchPosition).in(Meters)
        );
    }

    private Angle inchesToRadians(Distance d) {
        d = d.minus(MIN_HEIGHT);
        return Radians.of(d.in(Meters) / DRUM_RADIUS.in(Meters));
    }

    private Distance radiansToInches(Angle a) {
        double d = a.in(Radians) * DRUM_RADIUS.in(Meters);
        return Meters.of(d).plus(MIN_HEIGHT);
    }

    public Command minHeight() {
        return this.runOnce(() -> {
                Distance height = MIN_HEIGHT;
                Angle r = inchesToRadians(height);
                io.setWinchPosition(r);
            });
    }

    public Command L1() {
        return this.runOnce(() -> {
                Distance height = MIN_HEIGHT.plus(Inches.of(6.5));
                Angle r = inchesToRadians(height);
                io.setWinchPosition(r);
            });
    }

    public Command L2() {
        return this.runOnce(() -> {
                Distance height = MIN_HEIGHT.plus(Inches.of(23.5));
                Angle r = inchesToRadians(height);
                io.setWinchPosition(r);
            });
    }

    public Command L3() {
        return this.runOnce(() -> {
                Distance height = MIN_HEIGHT.plus(Inches.of(39.5));
                Angle r = inchesToRadians(height);
                io.setWinchPosition(r);
            });
    }

    public Command L4() {
        return this.runOnce(() -> {
                Distance height = MIN_HEIGHT.plus(Inches.of(64));
                Angle r = inchesToRadians(height);
                io.setWinchPosition(r);
            });
    }

    public Command maxHeight() {
        return this.runOnce(() -> {
                Distance height = MAX_EXTENSION.plus(MIN_HEIGHT);
                Angle r = inchesToRadians(height);
                io.setWinchPosition(r);
            });
    }

    /**
     * Computes the estimated position of each elevator stage (stage 1, stage 2,
     * carriage + loader) given the current winch position. Useful for rendering the
     * robot model in simulation.
     *
     * A continuous elevator lifts stages in order of least weight. As of 2/1, for
     * us that would be stage 2, stage 1, then carriage. In order, each stage will
     * contribute as much height as it can to the lift before the next stage
     * engages.
     *
     * @returns Pose array of each elevator stage in the order: stage 1, stage 2,
     *          carriage
     */
    public Pose3d[] getElevatorPoses() {
        var height = radiansToInches(inputs.winchPosition).minus(MIN_HEIGHT);
        var heightInches = height.in(Inches);
        // contributes no height until above 26.25 inches and contributes up to 25.25
        // inches
        var stage1Contribution = heightInches < 26.25
            ? 0
            : heightInches < 51.5
                ? height.minus(Inches.of(26.25)).in(Meters)
                : Inches.of(25.25).in(Meters);
        // contributes up to 26.25 inches starting at height 0
        var stage2Contribution = heightInches < 26.25
            ? height.in(Meters)
            : Inches.of(26.25).in(Meters);
        // contributes no height until above 51.5 inches and contributes up to 25.25
        // inches
        var stage3Contribution = heightInches < 51.5
            ? 0
            : height.minus(Inches.of(51.5)).in(Meters);

        return new Pose3d[] {
            new Pose3d(0, 0, stage1Contribution, Rotation3d.kZero),
            new Pose3d(
                0,
                0,
                stage1Contribution + stage2Contribution,
                Rotation3d.kZero
            ),
            new Pose3d(
                0,
                0,
                stage1Contribution + stage2Contribution + stage3Contribution,
                Rotation3d.kZero
            ),
        };
    }
}
