package frc.robot.Robot25;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import java.util.Random;

public class AlgaeJuker {
  private final NetworkTable limelight;
  private final MotorController leftMotor;
  private final MotorController rightMotor;
  private final Timer timer;
  private final Random rand;

  private double nextMoveTime = 0.0;
  private String currentMove = "forward";

  private static final double BASE_SPEED = 0.5;

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

    SmartDashboard.putString("Detected", detectedClass);
    SmartDashboard.putNumber("tx", tx);

    if (detectedClass.equals("algae")) {
      jukeMove(tx);
    } else if (detectedClass.equals("coral")) {
      pushForward();
    } else {
      searchForTarget();
    }
  }

  /** Tries to "outsmart" the algae by making quick jukes */
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
        SmartDashboard.putString("Action", "Juking left");
        break;
      case "fakeRight":
        leftMotor.set(0.2);
        rightMotor.set(0.6);
        SmartDashboard.putString("Action", "Juking right");
        break;
      case "swerve":
        if (tx > 0) {
          leftMotor.set(0.5);
          rightMotor.set(0.3);
        } else {
          leftMotor.set(0.3);
          rightMotor.set(0.5);
        }
        SmartDashboard.putString("Action", "Swerve around algae");
        break;
    }
  }

  /** Moves toward coral confidently */
  private void pushForward() {
    leftMotor.set(BASE_SPEED);
    rightMotor.set(BASE_SPEED);
    SmartDashboard.putString("Action", "Advancing toward coral");
  }

  /** Looks around if nothing is seen */
  private void searchForTarget() {
    leftMotor.set(0.4);
    rightMotor.set(-0.4);
    SmartDashboard.putString("Action", "Searching...");
  }
}
