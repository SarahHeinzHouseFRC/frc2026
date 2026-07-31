package frc.robot.testmode;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;

/** Shared timing, fault collection, and alert behavior for one subsystem test. */
public abstract class SubsystemTestCommand extends Command {
  static final String ALERT_GROUP = "testmode";

  private final String subsystemName;
  private final Alert progressAlert;
  private final Alert passAlert;
  private final Alert failureAlert;
  private final Set<String> failures = new LinkedHashSet<>();

  private FaultCollector faultCollector;
  private double phaseStart;
  private boolean complete;

  private String debugString = "";

  protected SubsystemTestCommand(String subsystemName, Subsystem... requirements) {
    this.subsystemName = subsystemName;
    progressAlert =
        new Alert(
            ALERT_GROUP, "[TESTMODE] Waiting to test " + subsystemName, Alert.AlertType.kInfo);
    passAlert =
        new Alert(
            ALERT_GROUP, "[" + subsystemName + "] Tests passed", Alert.AlertType.kInfo);
    failureAlert =
        new Alert(
            ALERT_GROUP, "[" + subsystemName + "] Tests failed", Alert.AlertType.kError);
    addRequirements(requirements);
  }

  @Override
  public final void initialize() {
    clearResultAlerts();
    failures.clear();
    faultCollector = null;
    complete = false;
    progressAlert.set(true);
    initializeTest();
  }

  @Override
  public final void execute() {
    if (faultCollector != null) {
      faultCollector.sample();
    }
    executeTest();
  }

  @Override
  public final boolean isFinished() {
    return complete;
  }

  @Override
  public final void end(boolean interrupted) {
    if (faultCollector != null) {
      faultCollector.sample();
    }
    stopTest();
    progressAlert.set(false);
    if (interrupted) {
      addFailure("test incomplete: robot disabled or left Test mode");
    }
    publishResult();
    faultCollector = null;
  }

  protected abstract void initializeTest();

  protected abstract void executeTest();

  protected abstract void stopTest();

  protected final void startFaultMonitoring(List<TestMotor> motors) {
    faultCollector = new FaultCollector(motors, failures);
    faultCollector.baselineStickyFaults();
  }

  protected final void startPhase(String status) {
    phaseStart = Timer.getFPGATimestamp();
    progressAlert.setText("[TESTMODE] Testing " + subsystemName + ": " + status);
  }

  protected final double phaseElapsed() {
    return Timer.getFPGATimestamp() - phaseStart;
  }

  protected final void addFailure(String failure) {
    failures.add(failure);
  }

  protected final void completeTest() {
    complete = true;
  }

  final void clearResultAlerts() {
    passAlert.set(false);
    failureAlert.set(false);
  }

  private void publishResult() {
    String assembledDebugString = debugString != null && !debugString.isEmpty() ? " (" + debugString + ")" : "";

    clearResultAlerts();
    if (failures.isEmpty()) {
      passAlert.setText("[" + subsystemName + "] Tests passed" + assembledDebugString);
      passAlert.set(true);
    } else {
      failureAlert.setText(
          "[" + subsystemName + "] Tests failed" + assembledDebugString + ": " + String.join("; ", failures));
      failureAlert.set(true);
    }
  }

  protected final void setDebugString(String debugString) {
    this.debugString = debugString;
  }

  protected static boolean withinTolerance(
      double measurement, double target, double tolerance) {
    return Math.abs(measurement - target) <= tolerance;
  }

}
