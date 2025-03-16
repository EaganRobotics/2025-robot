package frc.lib;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public final class SimTestRunner {

  private static final List<Function<RobotContainer, Command>> testCommands = new ArrayList<>();

  private SimTestRunner() {
  }

  public static void main(String[] args) throws NoSuchFieldException, IllegalAccessException {
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
    // TODO set test command on RobotContainer
    RobotBase.startRobot(() -> robot);
  }

  private static void stopRobot() throws InterruptedException {
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
