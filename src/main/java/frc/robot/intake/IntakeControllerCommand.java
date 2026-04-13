package frc.robot.intake;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.overbumper.OverBumper;
import frc.robot.overbumper.ShakeCommand;
import frc.robot.shooter.Shooter;
import org.littletonrobotics.junction.Logger;

import java.util.function.BooleanSupplier;

public class IntakeControllerCommand extends Command {
  private final Intake intake;
  private final XboxController driverController;
  private final XboxController operatorController;
  private final BooleanSupplier shakeAllowed;
  private final Command shakeCommand;
  private boolean isShakeScheduled = false;
  private final CommandScheduler commandScheduler = CommandScheduler.getInstance();

  private final Debouncer flywheelGoodDebouncer = new Debouncer(.1, Debouncer.DebounceType.kRising);

  public IntakeControllerCommand(
      XboxController driver, XboxController operator, BooleanSupplier shakeAllowed, Intake intake) {
    addRequirements(intake);
    this.intake = intake;
    this.driverController = driver;
    this.operatorController = operator;
    this.shakeCommand = new ShakeCommand(OverBumper.getInstance());

    this.shakeAllowed = shakeAllowed;
  }

  @Override
  public void execute() {
    boolean intaking =
        (driverController.getRightTriggerAxis() > .1)
            || (operatorController.getLeftTriggerAxis() > .1);
    boolean shooting = (operatorController.getRightTriggerAxis() > .1);
    boolean outtaking = false;
    boolean unjamming = false;

    boolean shouldOnlyShootIfAtSpeed = true;

    boolean isFlywheelAtSpeed = Shooter.getInstance().isFlywheelAtSpeed();

    boolean flywheelGood = flywheelGoodDebouncer.calculate(isFlywheelAtSpeed);

    Logger.recordOutput("/Intake/flywheelGood", isFlywheelAtSpeed);
    Logger.recordOutput("/Intake/debouncedFlywheelGood", flywheelGood);

    if (shouldOnlyShootIfAtSpeed && !flywheelGood && shooting) {
      shooting = false;
      unjamming = true;
    }

    if (intaking && shooting) {
      intake.intakeAndShoot();
    } else if (intaking) {
      intake.intake();
    } else if (shooting) {
      intake.shoot();
    } else if (outtaking) {
      intake.outtake();
    } else if (unjamming) {
      intake.intake(.25);
    } else {
      intake.stop();
    }

    boolean shake = shooting && shakeAllowed.getAsBoolean();

    if (shake && !commandScheduler.isScheduled(shakeCommand)) {
      commandScheduler.schedule(shakeCommand);
    } else if (!shake && commandScheduler.isScheduled(shakeCommand)) {
      commandScheduler.cancel(shakeCommand);
    }
  }

  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }
}
