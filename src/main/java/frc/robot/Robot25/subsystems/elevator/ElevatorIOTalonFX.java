package frc.robot.Robot25.subsystems.elevator;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.lib.devices.DigitalInputWrapper;
import frc.lib.devices.TalonFXWrapper;
import frc.lib.devices.TalonFXWrapper.FollowerConfig;
import org.littletonrobotics.junction.Logger;


public class ElevatorIOTalonFX implements ElevatorIO {
  DigitalInputWrapper lowerLimitSwitch = new DigitalInputWrapper(3, "lowerLimit", false);

  TalonFXWrapper elevatorTalonFX;
  double gearRatio = 5;
  double P = 1;
  double I = 0;
  double D = 0;
  final int rightMotorID = 20;
  final int leftMotorID = 21;

  public ElevatorIOTalonFX() {
    elevatorTalonFX = new TalonFXWrapper(rightMotorID, "Elevator", false, NeutralModeValue.Brake,
        gearRatio, P, I, D, RotationsPerSecondPerSecond.of(0), RotationsPerSecond.of(0), false,
        false, Rotations.of(0), Rotations.of(0), new FollowerConfig(leftMotorID, true),
        Units.Seconds.of(3), Units.Amps.of(75), Units.RotationsPerSecond.of(0));
  }

  @Override
  public void setWinchOpenLoop(Voltage output) {
    elevatorTalonFX.setVoltageOut(output);
  }

  @Override
  public void setWinchPosition(Angle angle) {
    Logger.recordOutput("elevetorAngle", angle);
    elevatorTalonFX.setMotionMagicVoltage(angle);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.lowerLimit = lowerLimitSwitch.get();
    inputs.winchConnected = false;
    inputs.winchVelocity = elevatorTalonFX.getVelocity();
    inputs.winchPosition = elevatorTalonFX.getPosition();
    inputs.winchCurrent = elevatorTalonFX.getTorqueCurrent();
    inputs.winchAppliedVolts = Volts.of(0);
    // System.out.println(lowerLimitSwitch.get());
  }
}
