package frc.robot.Robot25.subsystems.outtake;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.devices.DigitalInputWrapper;
import frc.lib.devices.TalonFXWrapper;
import org.littletonrobotics.junction.Logger;

public class OuttakeIOTalonFX implements OuttakeIO {
  TalonFXWrapper outtakeTalonFX;
  TalonFXWrapper outtakeRollerFX;
  final int motorID = 22;
  final int rollerID = 23;
  final int outtakeWheelDiameter = 5;
  final int outtakeRollerDiameter = 4;

  DigitalInputWrapper inputSensor = new DigitalInputWrapper(5, "LoadSideSensor", true);
  DigitalInputWrapper outputSensor = new DigitalInputWrapper(0, "ScoreSideSensor", true);

  public OuttakeIOTalonFX() {
    outtakeTalonFX = new TalonFXWrapper(motorID, "Outtake", true, NeutralModeValue.Brake, 1, 1, 0,
        1, RotationsPerSecondPerSecond.of(1), RotationsPerSecond.of(1),
        // RotationsPerSecCubed.of(0),
        false, false, Rotations.of(0), Rotations.of(0), null, Units.Seconds.of(1),
        Units.Amps.of(75), Units.RotationsPerSecond.of(1));
    outtakeRollerFX = new TalonFXWrapper(rollerID, "Outtake Roller", true, NeutralModeValue.Brake);
  }

  @Override
  public void setOpenLoop(Voltage output) {
    outtakeTalonFX.setVoltageOut(output);
  }

  public void setRollerOpenLoop(Voltage OuttakeOutput) {
    Voltage RollerOutput = OuttakeOutput.times(outtakeWheelDiameter / outtakeRollerDiameter);
    outtakeTalonFX.setVoltageOut(OuttakeOutput);
    outtakeRollerFX.setVoltageOut(RollerOutput);
  }

  @Override
  public void updateInputs(OuttakeIOInputs inputs) {
    inputs.outtakeConnected = false;
    inputs.outtakeVelocity = outtakeTalonFX.getVelocity();
    inputs.outtakeCurrent = outtakeTalonFX.getTorqueCurrent();
    inputs.outtakeAppliedVolts = Volts.of(0);
    inputs.seesCoralAtInput = inputSensor.get();
    inputs.seesCoralAtOutput = outputSensor.get();
  }

  @Override
  public void zeroMotor() {
    outtakeTalonFX.setPosition(0);
  }

  @Override
  public void setPosition(Angle angle) {
    Logger.recordOutput("outtakeAngle", angle);
    outtakeTalonFX.setMotionMagicVoltage(angle);
  }


}
