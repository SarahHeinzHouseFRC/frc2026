package frc.robot.testmode;

/** Read-only fault access used by the robot's test-mode diagnostics. */
public interface TestMotor {
  String name();

  int canId();

  int activeFaultBits();

  int stickyFaultBits();
}
