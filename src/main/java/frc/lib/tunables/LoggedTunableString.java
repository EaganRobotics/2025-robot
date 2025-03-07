package frc.lib.tunables;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

import org.littletonrobotics.junction.networktables.LoggedNetworkString;

import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;

public class LoggedTunableString extends LoggedNetworkString {

  private StringEntry entry;
  private List<Consumer<String>> listeners;

  public LoggedTunableString(String key, String defaultValue) {
    super(key, defaultValue);
    entry = NetworkTableInstance.getDefault().getStringTopic(key).getEntry(defaultValue,
        PubSubOption.keepDuplicates(false), PubSubOption.pollStorage(1));
    listeners = new ArrayList<>();
  }

  public void addListener(Consumer<String> listener) {
    listeners.add(listener);
    listener.accept(entry.get());
  }

  public void removeListener(Consumer<String> listener) {
    listeners.remove(listener);
  }

  @Override
  public void periodic() {
    super.periodic();
    var changes = entry.readQueueValues();
    if (changes.length == 1) {
      var newValue = changes[0];
      for (var listener : listeners) {
        listener.accept(newValue);
      }
    }
  }
}
