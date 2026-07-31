package frc.robot.testmode;

import frc.robot.subsystems.launcher.LauncherSubsystem;

import java.util.Arrays;
import java.util.stream.Collectors;

/** Checks both launcher flywheels at each required velocity. */
public final class LauncherTestCommand extends SubsystemTestCommand {
  private static final int[] SETPOINTS_RPM = {2000, 3000, 4000, 5000};
  private static final int RPM_TOLERANCE = 100;

  private double[] maxSetpointDeviations = {0.0, 0.0, 0.0, 0.0};

  private final LauncherSubsystem launcher;
  private int setpointIndex;

  public LauncherTestCommand(LauncherSubsystem launcher) {
    super("LAUNCHER", launcher);
    this.launcher = launcher;
  }

  @Override
  protected void initializeTest() {
    stopTest();
    maxSetpointDeviations = new double[]{0.0, 0.0, 0.0, 0.0};
    startFaultMonitoring(launcher.getTestMotors());
    setpointIndex = 0;
    commandCurrentSetpoint();
  }

  @Override
  protected void executeTest() {
    double elapsed = phaseElapsed();
    if (elapsed >= 1.5) {
      checkSetpoint(setpointIndex);
    }
    if (elapsed >= 2.5) {
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

  private void checkSetpoint(int setpointIndex) {
    int setpoint = SETPOINTS_RPM[setpointIndex];
    double[] velocities = launcher.getFlywheelVelocities();
    for (int i = 0; i < velocities.length; i++) {
      if (!withinTolerance(Math.abs(velocities[i]), setpoint, RPM_TOLERANCE)) {
        addFailure("flywheel " + (i + 1) + " outside tolerance at " + setpoint + " RPM");
      }

      double deviation = Math.abs(Math.abs(velocities[i]) - setpoint);
      if (deviation > maxSetpointDeviations[setpointIndex]) {
        maxSetpointDeviations[setpointIndex] = deviation;
      }
    }
  }

  @Override
  protected void stopTest() {
    String debugString = Arrays.stream(maxSetpointDeviations)
        .mapToObj(d -> String.format("%.0f", d))
        .collect(Collectors.joining("-"));
    setDebugString(debugString);
    launcher.stopFlywheel();
  }
}
