package frc.robot.Robot25.commands;

import java.util.Set;

import edu.wpi.first.hal.HALValue;
import edu.wpi.first.hal.simulation.NotifyCallback;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.RobotContainer;
import frc.robot.SimConstants;

public final class SimDriverPractice {

  // Private constructor to prevent instantiation
  private SimDriverPractice() {}

  public static Command simDriverPractice(RobotContainer robotContainer) {

    return Commands.defer(() -> {

      if (SimConstants.SIM_MODE != SimConstants.Mode.SIM) {
        throw new IllegalStateException("SimDriverPractice can only be used in simulation mode");
      }

      // CommandScheduler.getInstance().cancelAll();
      robotContainer.resetSimulation();

      var autoCommand = robotContainer.getAutonomousCommand().asProxy();

      return autoCommand.beforeStarting(() -> {
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setEventName("Sim Driver Practice");
        DriverStationSim.setGameSpecificMessage("Sim Driver Practice");
        DriverStationSim.setMatchType(MatchType.Practice);
        DriverStationSim.setMatchTime(15);
        DriverStationSim.notifyNewData();
        System.out.println(
            "========================================\n|          Autonomous started          |\n========================================");
      }).alongWith(Commands.waitSeconds(15)).withTimeout(15).andThen(() -> {
        DriverStationSim.setTest(false);
        DriverStationSim.setAutonomous(false);
        DriverStationSim.setEnabled(true);
        DriverStationSim.setMatchTime(0);
        DriverStationSim.notifyNewData();
        System.out.println(
            "========================================\n|   Autonomous over - TeleOp started   |\n========================================");

        var teleopPeriodCommand = Commands.waitSeconds(135).beforeStarting(() -> {
          DriverStationSim.setMatchTime(135);
          DriverStationSim.notifyNewData();
        }).andThen(() -> {
          DriverStationSim.setEnabled(false);
          DriverStationSim.setMatchTime(0);
          DriverStationSim.notifyNewData();
          System.out.println(
              "========================================\n|              TeleOp over             |\n========================================");
        });

        teleopPeriodCommand.schedule();
      });
    }, Set.of());
  }
}
