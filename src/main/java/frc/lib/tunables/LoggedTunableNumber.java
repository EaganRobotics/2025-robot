package frc.lib.tunables;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;

public class LoggedTunableNumber extends LoggedNetworkNumber {

  private DoubleEntry entry;
  private List<Consumer<Double>> listeners;

  public LoggedTunableNumber(String key, double defaultValue) {
    super(key, defaultValue);
    entry = NetworkTableInstance.getDefault().getDoubleTopic(key).getEntry(defaultValue,
        PubSubOption.keepDuplicates(false), PubSubOption.pollStorage(1));
    listeners = new ArrayList<>();
  }

  public void addListener(Consumer<Double> listener) {
    listeners.add(listener);
    listener.accept(entry.get());
  }

  public void removeListener(Consumer<Double> listener) {
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
