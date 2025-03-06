package frc.lib.tunables;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;

public class LoggedTunableBoolean extends LoggedNetworkBoolean {

  private BooleanEntry entry;
  private List<Consumer<Boolean>> listeners;

  public LoggedTunableBoolean(String key, boolean defaultValue) {
    super(key, defaultValue);
    entry = NetworkTableInstance.getDefault().getBooleanTopic(key).getEntry(defaultValue,
        PubSubOption.keepDuplicates(false), PubSubOption.pollStorage(1));
    listeners = new ArrayList<>();
  }

  public void addListener(Consumer<Boolean> listener) {
    listeners.add(listener);
    listener.accept(entry.get());
  }

  public void removeListener(Consumer<Boolean> listener) {
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
