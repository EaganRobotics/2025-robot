package frc.robot.Robot25.subsystems.outtake;

import static edu.wpi.first.units.Units.Volts;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.devices.DigitalInputWrapper;
import frc.lib.devices.TalonFXWrapper;

public class OuttakeIOTalonFX implements OuttakeIO {
  TalonFXWrapper outtakeTalonFX;
  final int motorID = 22;

  DigitalInputWrapper inputSensor = new DigitalInputWrapper(0, "LoadSideSensor", true);
  // DigitalInputWrapper outputSensor = new DigitalInputWrapper(0, "ScoreSideSensor", true);


  public OuttakeIOTalonFX() {
    outtakeTalonFX = new TalonFXWrapper(motorID, "Outtake", true, NeutralModeValue.Brake);
  }

  @Override
  public void setOpenLoop(Voltage output) {
    outtakeTalonFX.setVoltageOut(output);
  }

  @Override
  public void updateInputs(OuttakeIOInputs inputs) {
    inputs.outtakeConnected = false;
    inputs.outtakeVelocity = outtakeTalonFX.getVelocity();
    inputs.outtakeCurrent = outtakeTalonFX.getTorqueCurrent();
    inputs.outtakeAppliedVolts = Volts.of(0);
    inputs.seesCoralAtInput = inputSensor.get();
    // inputs.seesCoralAtOutput = outputSensor.get();
  }

}
