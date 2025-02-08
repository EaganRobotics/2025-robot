package frc.robot.Robot25.subsystems.elevator;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.devices.TalonFXWrapper;
import frc.lib.devices.TalonFXWrapper.FollowerConfig;

public class ElevatorIOTalonFX implements ElevatorIO{
    TalonFXWrapper elevatorTalonFX;
    double gearRatio;
    double P;
    double I;
    double D;
    final int rightMotorID = 20;
    final int leftMotorID = 21;

    public ElevatorIOTalonFX() {
        elevatorTalonFX = new TalonFXWrapper(
                rightMotorID,
                "Elevator",
                true,
                NeutralModeValue.Brake,
                gearRatio,
                P,
                I,
                D,
                RotationsPerSecondPerSecond.of(0),
                RotationsPerSecond.of(0),
                false,
                false,
                Rotations.of(120.0 / 360.0),
                Rotations.of(0),
                new FollowerConfig(leftMotorID, true),
                Units.Seconds.of(3),
                Units.Amps.of(75),
                Units.RotationsPerSecond.of(1));
    }

    @Override
    public void setWinchOpenLoop(Voltage output) {
        elevatorTalonFX.setVoltageOut(output);
    }

    @Override
    public void setWinchPosition(Angle angle) {
        elevatorTalonFX.setPosition(angle.in(Rotations));
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.winchConnected = false;
        inputs.winchVelocity = elevatorTalonFX.getVelocity();
        inputs.winchPosition = elevatorTalonFX.getPosition();
        inputs.winchCurrent = elevatorTalonFX.getTorqueCurrent();
        inputs.winchAppliedVolts = Volts.of(0);
    }
}
