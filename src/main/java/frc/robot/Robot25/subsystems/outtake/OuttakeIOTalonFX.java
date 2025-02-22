package frc.robot.Robot25.subsystems.outtake;

import static edu.wpi.first.units.Units.Volts;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.devices.DigitalInputWrapper;
import frc.lib.devices.TalonFXWrapper;

public class OuttakeIOTalonFX implements OuttakeIO {
  TalonFXWrapper outtakeTalonFX;
  TalonFXWrapper outtakeRollerFX;
  final int motorID = 22;
  final int rollerID = 23;
  final int outtakeWheelDiameter = 3;
  final int outtakeRollerDiameter = 2;

  // DigitalInputWrapper inputSensor = new DigitalInputWrapper(0, "LoadSideSensor", true);
  DigitalInputWrapper outputSensor = new DigitalInputWrapper(0, "ScoreSideSensor", true);


  public OuttakeIOTalonFX() {
    outtakeTalonFX = new TalonFXWrapper(motorID, "Outtake", true, NeutralModeValue.Brake);
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
    // inputs.seesCoralAtInput = inputSensor.get();
    inputs.seesCoralAtOutput = outputSensor.get();
  }

}
