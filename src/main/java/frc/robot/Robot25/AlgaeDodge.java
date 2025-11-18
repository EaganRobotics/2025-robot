package frc.robot.Robot25;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import org.littletonrobotics.junction.Logger;

public class AlgaeDodge {
  private final NetworkTable limelight;
  private final MotorController leftMotor;
  private final MotorController rightMotor;

  private static final double DODGE_SPEED = 0.4;

  public AlgaeDodge() {
    limelight = NetworkTableInstance.getDefault().getTable("limelight");
    leftMotor = new PWMSparkMax(0);
    rightMotor = new PWMSparkMax(1);
  }

  public void update() {
    // examples: "tclass" (string) or "tclassID" (number)
    String detectedClass = limelight.getEntry("tclass").getString("none");
    double tx = limelight.getEntry("tx").getDouble(0.0); // horizontal offset
    double ta = limelight.getEntry("ta").getDouble(0.0); // target area (optional)

    Logger.recordOutput("Detected", detectedClass);
    Logger.recordOutput("tx", tx);
    Logger.recordOutput("ta", ta);

    if (detectedClass.equals("algae") || detectedClass.equals("algae?")) {
      // turns right
      leftMotor.set(DODGE_SPEED);
      rightMotor.set(-DODGE_SPEED);
      Logger.recordOutput("Action", "Dodging algae");
    } else {
      leftMotor.stopMotor();
      rightMotor.stopMotor();
      Logger.recordOutput("Action", "Idle");
    }
  }
}
