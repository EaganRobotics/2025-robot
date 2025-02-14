package frc.robot.Robot25.subsystems.outtake;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Outtake extends SubsystemBase {
  private final OuttakeIO io;
  private final OuttakeIOInputsAutoLogged inputs = new OuttakeIOInputsAutoLogged();

  public Outtake(OuttakeIO io) {
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
    });
  }

  public Command autoQueueCoral() {
    return this.run(() -> {
      if (inputs.seesCoralAtOutput) {
        io.setOpenLoop(Volts.of(0));
      } else if (inputs.seesCoralAtInput) {
        io.setOpenLoop(Volts.of(6));
      } else {
        io.setOpenLoop(Volts.of(12));
      }
    });
  }

  public Command autoWaitUntilCoral() {

    return this.run(() -> {
      io.setOpenLoop(Volts.of(6));
    }).until(() -> {
      return inputs.seesCoralAtOutput;
    }).withTimeout(2).andThen(this.runOnce(() -> {
      io.setOpenLoop(Volts.of(0));

    }));


    // return Commands.waitUntil(() -> {

    // return inputs.seesCoralAtInput;
    // });

  }
}
