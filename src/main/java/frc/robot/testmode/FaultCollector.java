package frc.robot.testmode;

import java.util.List;
import java.util.Set;

/** Retains transient active faults and sticky faults added after a test begins. */
final class FaultCollector {
  private static final String[] FAULT_NAMES = {
    "other", "motor type", "sensor", "CAN", "temperature", "gate driver", "ESC EEPROM", "firmware"
  };

  private final List<TestMotor> motors;
  private final int[] initialStickyFaults;
  private final Set<String> failures;

  FaultCollector(List<TestMotor> motors, Set<String> failures) {
    this.motors = List.copyOf(motors);
    this.failures = failures;
    initialStickyFaults = new int[motors.size()];
  }

  void baselineStickyFaults() {
    for (int i = 0; i < motors.size(); i++) {
      initialStickyFaults[i] = motors.get(i).stickyFaultBits();
    }
  }

  void sample() {
    for (int i = 0; i < motors.size(); i++) {
      TestMotor motor = motors.get(i);
      addFaults(motor, "active", motor.activeFaultBits());
      addFaults(motor, "new sticky", motor.stickyFaultBits() & ~initialStickyFaults[i]);
    }
  }

  private void addFaults(TestMotor motor, String kind, int bits) {
    for (int bit = 0; bit < FAULT_NAMES.length; bit++) {
      if ((bits & (1 << bit)) != 0) {
        failures.add(
            "%s (CAN %d) %s fault: %s"
                .formatted(motor.name(), motor.canId(), kind, FAULT_NAMES[bit]));
      }
    }
  }
}
