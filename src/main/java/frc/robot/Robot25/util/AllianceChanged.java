package frc.robot.Robot25.util;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public final class AllianceChanged {
  private static Optional<Alliance> alliance = Optional.empty();

  public static final Trigger allianceChanged = new Trigger(() -> {
    var newAlliance = DriverStation.getAlliance();
    if (!alliance.equals(newAlliance)) {
      alliance = newAlliance;
      return true;
    }
    return false;
  });
}
