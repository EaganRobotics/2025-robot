package frc.robot.Robot25;

import frc.robot.Robot25.subsystems.drive.Drive;

public final class RobotContainerProvider {
  private static final RobotContainer get2025RobotContainer(frc.lib.RobotContainer robotContainer) {
    if (!(robotContainer instanceof RobotContainer)) {
      throw new IllegalArgumentException(
          "frc.robot.Robot25.RobotContainerProvider can only be used with the 2025 RobotContainer");
    }

    return (RobotContainer) robotContainer;
  }

  public static Drive getDriveSubsystem(frc.lib.RobotContainer robotContainer) {
    var robot = get2025RobotContainer(robotContainer);

    try {
      var driveField = RobotContainer.class.getDeclaredField("drive");
      driveField.setAccessible(true);
      return (Drive) driveField.get(robot);
    } catch (NoSuchFieldException | SecurityException | IllegalAccessException e) {
      throw new RuntimeException("Failed to get drive subsystem", e);
    }
  }
}
