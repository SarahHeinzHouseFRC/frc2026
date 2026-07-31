package frc.robot.testmode;

import frc.robot.subsystems.launcher.LauncherSubsystem;

/** Checks both launcher flywheels at each required velocity. */
public final class LauncherTestCommand extends SubsystemTestCommand {
  private static final int[] SETPOINTS_RPM = {2000, 3000, 4000, 5000};

  private final LauncherSubsystem launcher;
  private int setpointIndex;

  public LauncherTestCommand(LauncherSubsystem launcher) {
    super("LAUNCHER", launcher);
    this.launcher = launcher;
  }

  @Override
  protected void initializeTest() {
    stopTest();
    startFaultMonitoring(launcher.getTestMotors());
    setpointIndex = 0;
    commandCurrentSetpoint();
  }

  @Override
  protected void executeTest() {
    double elapsed = phaseElapsed();
    int setpoint = SETPOINTS_RPM[setpointIndex];
    if (elapsed >= 1.5) {
      checkSetpoint(setpoint);
    }
    if (elapsed >= 2.0) {
      setpointIndex++;
      if (setpointIndex >= SETPOINTS_RPM.length) {
        stopTest();
        completeTest();
      } else {
        commandCurrentSetpoint();
      }
    }
  }

  private void commandCurrentSetpoint() {
    int setpoint = SETPOINTS_RPM[setpointIndex];
    launcher.setFlywheelSetpoint(setpoint);
    startPhase(setpoint + " RPM");
  }

  private void checkSetpoint(int setpoint) {
    double tolerance = Math.max(100.0, setpoint * 0.05);
    double[] velocities = launcher.getFlywheelVelocities();
    for (int i = 0; i < velocities.length; i++) {
      if (!withinTolerance(Math.abs(velocities[i]), setpoint, tolerance)) {
        addFailure("flywheel " + (i + 1) + " outside tolerance at " + setpoint + " RPM");
      }
    }
  }

  @Override
  protected void stopTest() {
    launcher.stopFlywheel();
  }
}
