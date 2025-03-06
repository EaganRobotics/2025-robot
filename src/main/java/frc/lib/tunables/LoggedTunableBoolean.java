package frc.lib.tunables;

import java.lang.ref.WeakReference;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.MatchType;

public class LoggedTunableBoolean extends LoggedNetworkBoolean implements LoggedTunable<Boolean> {

  private final boolean defaultValue;
  private final BooleanEntry entry;
  private final List<WeakReference<Consumer<Boolean>>> listeners;

  public Boolean getDefaultValue() {
    return defaultValue;
  }

  public Boolean getValue() {
    return entry.get();
  }

  public List<WeakReference<Consumer<Boolean>>> getListeners() {
    return listeners;
  }

  public LoggedTunableBoolean(String key, boolean defaultValue) {
    super(key, defaultValue);
    this.defaultValue = defaultValue;
    entry = NetworkTableInstance.getDefault().getBooleanTopic(key).getEntry(defaultValue,
        PubSubOption.keepDuplicates(false), PubSubOption.pollStorage(1));
    listeners = new ArrayList<>();
  }

  @Override
  public void periodic() {
    super.periodic();

    // Only do tunables when not in a match
    if (DriverStation.getMatchType() == MatchType.None) {
      var changes = entry.readQueueValues();
      if (changes.length == 1) {
        notifyListeners();
      }
    }
  }
}
