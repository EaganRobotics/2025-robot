package frc.robot.Robot25;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
    // Limelight posts class IDs or labels depending on your model
    // Example keys: "tclass" (string) or "tclassID" (number)
    String detectedClass = limelight.getEntry("tclass").getString("none");
    double tx = limelight.getEntry("tx").getDouble(0.0); // horizontal offset
    double ta = limelight.getEntry("ta").getDouble(0.0); // target area (optional)

    SmartDashboard.putString("Detected", detectedClass);
    SmartDashboard.putNumber("tx", tx);
    SmartDashboard.putNumber("ta", ta);

    if (detectedClass.equals("algae")) {
      // Turn right to dodge
      leftMotor.set(DODGE_SPEED);
      rightMotor.set(-DODGE_SPEED);
      SmartDashboard.putString("Action", "Dodging algae");
    } else if (detectedClass.equals("coral")) {
      // Move forward toward coral
      leftMotor.set(0.5);
      rightMotor.set(0.5);
      SmartDashboard.putString("Action", "Approaching coral");
    } else {
      // Stop if no detection
      leftMotor.stopMotor();
      rightMotor.stopMotor();
      SmartDashboard.putString("Action", "Idle");
    }
  }
}
