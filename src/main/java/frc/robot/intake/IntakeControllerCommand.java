package frc.robot.intake;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.overbumper.OverBumper;
import frc.robot.overbumper.ShakeCommand;
import frc.robot.shooter.Shooter;

import java.util.function.BooleanSupplier;

public class IntakeControllerCommand extends Command {
  private final Intake intake;
  private final XboxController driverController;
  private final XboxController operatorController;
  private final BooleanSupplier shakeAllowed;
  private final Command shakeCommand;
  private boolean isShakeScheduled = false;
  private final CommandScheduler commandScheduler = CommandScheduler.getInstance();

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

    boolean shouldOnlyShootIfAtSpeed = true;

    if (shouldOnlyShootIfAtSpeed && !Shooter.getInstance().isFlywheelAtSpeed()) {
      shooting = false;
    }

    if (intaking && shooting) {
      intake.intakeAndShoot();
    } else if (intaking) {
      intake.intake();
    } else if (shooting) {
      intake.shoot();
    } else if (outtaking) {
      intake.outtake();
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
