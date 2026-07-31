package frc.robot.testmode;

import java.util.List;
import java.util.Set;

/** Retains transient active diagnostics and sticky diagnostics added after a test begins. */
final class FaultCollector {
  private static final String[] FAULT_NAMES = {
    "other", "motor type", "sensor", "CAN", "temperature", "gate driver", "ESC EEPROM", "firmware"
  };
  private static final String[] WARNING_NAMES = {
    "brownout", "overcurrent", "ESC EEPROM", "external EEPROM", "sensor", "stall", "has reset", "other"
  };

  private final List<TestMotor> motors;
  private final int[] initialStickyFaults;
  private final int[] initialStickyWarnings;
  private final Set<String> failures;

  FaultCollector(List<TestMotor> motors, Set<String> failures) {
    this.motors = List.copyOf(motors);
    this.failures = failures;
    initialStickyFaults = new int[motors.size()];
    initialStickyWarnings = new int[motors.size()];
  }

  void baselineStickyFaults() {
    for (int i = 0; i < motors.size(); i++) {
      initialStickyFaults[i] = motors.get(i).stickyFaultBits();
      initialStickyWarnings[i] = motors.get(i).stickyWarningBits();
    }
  }

  void sample() {
    for (int i = 0; i < motors.size(); i++) {
      TestMotor motor = motors.get(i);
      addBits(motor, "active", "fault", FAULT_NAMES, motor.activeFaultBits());
      addBits(
          motor,
          "new sticky",
          "fault",
          FAULT_NAMES,
          motor.stickyFaultBits() & ~initialStickyFaults[i]);
      addBits(motor, "active", "warning", WARNING_NAMES, motor.activeWarningBits());
      addBits(
          motor,
          "new sticky",
          "warning",
          WARNING_NAMES,
          motor.stickyWarningBits() & ~initialStickyWarnings[i]);
    }
  }

  private void addBits(TestMotor motor, String kind, String type, String[] names, int bits) {
    for (int bit = 0; bit < names.length; bit++) {
      if ((bits & (1 << bit)) != 0) {
        failures.add(
            "%s (CAN %d) %s %s: %s"
                .formatted(motor.name(), motor.canId(), kind, type, names[bit]));
      }
    }
  }
}
