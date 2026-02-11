package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class WatchedLoggedNetworkNumber extends LoggedNetworkNumber {
  private double lastValue = 0.0;
  private boolean changed = false;

  public WatchedLoggedNetworkNumber(String key) {
    super(key);
  }

  public WatchedLoggedNetworkNumber(String key, double defaultValue) {
    super(key, defaultValue);
    lastValue = defaultValue;
  }

  @Override
  public void periodic() {
    changed = get() != lastValue;
    lastValue = get();
    super.periodic();
  }

  public boolean isUpdated() {
    return changed;
  }
}
