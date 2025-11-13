package frc.robot.Robot25;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.Timer;
import java.util.Random;
import org.littletonrobotics.junction.Logger;

public class AlgaeJuker {
  private final NetworkTable limelight;
  private final MotorController leftMotor;
  private final MotorController rightMotor;
  private final Timer timer;
  private final Random rand;

  private double nextMoveTime = 0.0;
  private String currentMove = "forward";

  public AlgaeJuker() {
    limelight = NetworkTableInstance.getDefault().getTable("limelight");
    leftMotor = new PWMSparkMax(0);
    rightMotor = new PWMSparkMax(1);
    timer = new Timer();
    rand = new Random();
    timer.start();
  }

  public void update() {
    String detectedClass = limelight.getEntry("tclass").getString("none");
    double tx = limelight.getEntry("tx").getDouble(0.0);

    Logger.recordOutput("Detected", detectedClass);
    Logger.recordOutput("tx", tx);

    if (detectedClass.equals("algae")) {
      jukeMove(tx);
    } else {
      searchForTarget();
    }
  }

  private void jukeMove(double tx) {
    if (timer.get() > nextMoveTime) {
      int moveType = rand.nextInt(3);
      if (moveType == 0)
        currentMove = "fakeLeft";
      else if (moveType == 1)
        currentMove = "fakeRight";
      else
        currentMove = "swerve";
      nextMoveTime = timer.get() + 0.8 + rand.nextDouble() * 0.7;
    }

    switch (currentMove) {
      case "fakeLeft":
        leftMotor.set(0.6);
        rightMotor.set(0.2);
        Logger.recordOutput("Action", "Juking left");
        break;
      case "fakeRight":
        leftMotor.set(0.2);
        rightMotor.set(0.6);
        Logger.recordOutput("Action", "Juking right");
        break;
      case "swerve":
        if (tx > 0) {
          leftMotor.set(0.5);
          rightMotor.set(0.3);
        } else {
          leftMotor.set(0.3);
          rightMotor.set(0.5);
        }
        Logger.recordOutput("Action", "Swerve around algae");
        break;
    }
  }

  private void searchForTarget() {
    leftMotor.set(0.4);
    rightMotor.set(-0.4);
    Logger.recordOutput("Action", "Searching...");
  }
}
