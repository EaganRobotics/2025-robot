package frc.lib;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.controllers.CommandXBoxWrapper;
import frc.robot.Robot;
import frc.robot.Robot25.RobotContainerProvider;
import frc.robot.Robot25.commands.DriveCommands;

public final class SimTestRunner {

  private static final List<Function<RobotContainer, Command>> testCommands = new ArrayList<>();

  private SimTestRunner() {
  }

  public static void main(String[] args) throws NoSuchFieldException, IllegalAccessException {
    // create an example test command
    // var exampleTest = new IntegrationTest() {
    // @Override
    // public Command testCommand(RobotContainer robotContainer) {
    // var drive = RobotContainerProvider.getDriveSubsystem(robotContainer);

    // return DriveCommands.joystickDrive(drive, () -> 0.2, () -> 0.0, () -> 0.0)
    // .withDeadline(Commands.waitSeconds(2));
    // }
    // };
    var exampleTest = new IntegrationTest() {
      @Override
      public Command testCommand(RobotContainer robotContainer) {
        var xb = new MockXboxController(0);
        var xbc = new XboxController(0);
        return Commands
            .run(() -> System.out.println("XB Controller LY: " + xbc.getLeftY() + ", LX: " + xbc.getLeftX() + ", RX: "
                + xbc.getRightX()))
            .withTimeout(7)
            .alongWith(Commands.print("\n\n----------------------Starting test----------------------\n\n")
                .andThen(Commands.waitSeconds(1)).andThen(xb.lerpLeftY(5,
                    0, 0.25, 0.5, 0.5, 0.25, 0).alongWith(
                        xb.lerpLeftX(5,
                            0, 0.25, 0.5, 0, -0.5, -0.5, -0.25, 0)
                            .alongWith(Commands.waitSeconds(1).andThen(
                                xb.lerpRightX(4, 0, 0.5, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0.5, 0.25, 0.2, 0.15,
                                    0)))))
                .andThen(
                    Commands.print("\n\n----------------------Test complete----------------------\n\n")));
      }
    };

    for (var testCommandSupplier : testCommands) {
      runTest(testCommandSupplier);
    }
  }

  private static void runTest(Function<RobotContainer, Command> testCommandSupplier)
      throws NoSuchFieldException, IllegalAccessException {
    var robot = new Robot();
    var robotContainerField = Robot.class.getDeclaredField("robotContainer");
    robotContainerField.setAccessible(true);
    var robotContainer = (RobotContainer) robotContainerField.get(robot);

    // Create the test command and chain it with a stop robot command
    var testCommand = testCommandSupplier.apply(robotContainer);
    testCommand = testCommand.finallyDo(() -> {
      try {
        stopRobot();
      } catch (InterruptedException e) {
        System.err.println("Failed to stop robot within 5 seconds");
        System.exit(1);
      }
    });

    // Send the test command to the robot and start it
    DriverStationSim.setDsAttached(true);
    // DriverStationSim.setTest(true);
    DriverStationSim.setEnabled(true);
    var testCommandField = Robot.class.getDeclaredField("testCommand");
    testCommandField.setAccessible(true);
    testCommandField.set(robot, testCommand);
    RobotBase.startRobot(() -> robot);
  }

  private static void stopRobot() throws InterruptedException {
    System.exit(0);
    var robotThreadId = RobotBase.getMainThreadId();
    var threads = new Thread[Thread.activeCount()];
    Thread.enumerate(threads);
    for (var thread : threads) {
      if (thread.threadId() == robotThreadId) {
        thread.interrupt();
        thread.join(5000);
      }
    }
  }

  public static abstract class IntegrationTest {

    public IntegrationTest() {
      testCommands.add(this::testCommand);
    }

    public abstract Command testCommand(RobotContainer robotContainer);
  }
}
