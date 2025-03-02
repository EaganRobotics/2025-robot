package frc.robot.Robot25.subsystems.AlgaeEater;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot25.subsystems.outtake.OuttakeIO;
import frc.robot.Robot25.subsystems.outtake.OuttakeIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class Algae extends SubsystemBase {
  
  private final AlgaeIO io;
  private final AlgaeIOInputsAutoLogged inputs = new OuttakeIOInputsAutoLogged();

  public Algae(AlgaeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Outtake", inputs);
  }

  public Command setOpenLoop(Voltage output) {
    return this.startEnd(() -> {
      io.setOpenLoop(output);
    }, () -> {
      io.setOpenLoop(Volts.of(0));
    }

    );
  }


}
