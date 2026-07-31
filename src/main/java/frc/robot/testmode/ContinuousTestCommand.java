package frc.robot.testmode;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;

/** Continuously evaluates a non-blocking condition while other subsystem tests run. */
public final class ContinuousTestCommand extends Command {
  private final BooleanSupplier condition;
  private final Alert passAlert;
  private final Alert failureAlert;

  private boolean everFailed;

  public ContinuousTestCommand(String name, BooleanSupplier condition) {
    this.condition = condition;
    passAlert =
        new Alert(
            SubsystemTestCommand.ALERT_GROUP,
            "[" + name + "] Test passed",
            Alert.AlertType.kInfo);
    failureAlert =
        new Alert(
            SubsystemTestCommand.ALERT_GROUP,
            "[" + name + "] Test failed",
            Alert.AlertType.kError);
  }

  @Override
  public void initialize() {
    everFailed = false;
    passAlert.set(false);
    failureAlert.set(false);
    sample();
  }

  @Override
  public void execute() {
    sample();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    passAlert.set(!everFailed);
    failureAlert.set(everFailed);
  }

  private void sample() {
    if (!condition.getAsBoolean()) {
      everFailed = true;
    }
  }
}
