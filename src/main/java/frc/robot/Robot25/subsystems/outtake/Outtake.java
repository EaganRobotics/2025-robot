package frc.robot.Robot25.subsystems.outtake;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;
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
    }

    );
  }

  public Command setRopenLoop(Voltage output) {
    return this.startEnd(() -> {
      io.setRollerOpenLoop(output);
    }, () -> {
      io.setRollerOpenLoop(Volts.of(0));
    }

    );
  }

  public Command setOpenLop(Voltage output) {
    return this.startEnd(() -> {
      io.setOpenLoop(output);
    }, () -> {
      io.setOpenLoop(Volts.of(0));
    }).withTimeout(2);
  }

  public Command autoQueueCoral() {
    return this.runEnd(() -> {
      Logger.recordOutput("Outtake/AutoQueuing", true);
      if (inputs.seesCoralAtOutput && inputs.seesCoralAtInput) {
        io.setRollerOpenLoop(Volts.of(0));
      } else if (!inputs.seesCoralAtOutput && inputs.seesCoralAtInput) {
        io.setRollerOpenLoop(Volts.of(7));
      } else if (inputs.seesCoralAtOutput && !inputs.seesCoralAtInput) {
        io.setRollerOpenLoop(Volts.of(0));
      } else { // !!
        io.setRollerOpenLoop(Volts.of(8));
      }
    }, () -> {
      Logger.recordOutput("Outtake/AutoQueuing", false);
      io.setRollerOpenLoop(Volts.of(0));
    });
  }

  public Command autoQueueCoralOveride() {
    return Commands.run(() -> io.setRollerOpenLoop(Volts.of(0))).withName("Outtake.autoQueueCoralOveride");
  }

  public Command autoQueueCoral2() {
    return this.runEnd(() -> {
      Logger.recordOutput("Outtake/AutoQueuing", true);
      if (inputs.seesCoralAtOutput) {
        io.setRollerOpenLoop(Volts.of(0));
        // } else if (inputs.seesCoralAtInput) {
        // io.setOpenLoop(Volts.of(6));
      } else {
        io.setRollerOpenLoop(Volts.of(5));
      }
    }, () -> {
      Logger.recordOutput("Outtake/AutoQueuing", false);
      io.setRollerOpenLoop(Volts.of(0));
    }).withTimeout(4);
  }

  public Command depositCoral() {
    return setOpenLoop(Volts.of(6)).until(seesAtOutputTrigger.negate().debounce(0.1))
        .withTimeout(1);
  }

  public Command openLoop(DoubleSupplier speed) {
    return this.runEnd(() -> {
      io.setRollerOpenLoop(Volts.of(speed.getAsDouble() * Math.PI));
      // we are not convirting to raidans we just wanted pi (:
    }, () -> {
      io.setRollerOpenLoop(Volts.of(0));
    }).withName("Outtake.openLoop");
  }

  public final Trigger seesAtOutputTrigger = new Trigger(() -> inputs.seesCoralAtOutput);
  public final Trigger seesAtInputTrigger = new Trigger(() -> inputs.seesCoralAtInput);

  // public Command specialDepositCoral() {
  // return setOpenLop(Volts.of(5));
  // }

  public Command reverseCoral() {
    return setRopenLoop(Volts.of(-5)).withTimeout(1.25).withName("Outtake.reverseCoral");
  }
}
