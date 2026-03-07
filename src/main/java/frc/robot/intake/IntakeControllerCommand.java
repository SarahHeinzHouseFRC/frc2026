package frc.robot.intake;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.overbumper.OverBumper;
import frc.robot.overbumper.ShakeCommand;

public class IntakeControllerCommand extends Command {
  private final Intake intake;
  private final XboxController driverController;
  private final XboxController operatorController;
  private final Command shakeCommand;
  private boolean isShakeScheduled = false;
  private final CommandScheduler commandScheduler = CommandScheduler.getInstance();

  public IntakeControllerCommand(XboxController driver, XboxController operator, Intake intake) {
    addRequirements(intake);
    this.intake = intake;
    this.driverController = driver;
    this.operatorController = operator;
    this.shakeCommand = new ShakeCommand(OverBumper.getInstance());
  }

  @Override
  public void execute() {
    boolean intaking = driverController.getRightTriggerAxis() > .1;
    boolean shooting = (operatorController.getRightTriggerAxis() > .1);
    boolean outtaking = operatorController.getLeftTriggerAxis() > .1;

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

    if (shooting && !commandScheduler.isScheduled(shakeCommand)) {
      commandScheduler.schedule(shakeCommand);
    } else if (!shooting && commandScheduler.isScheduled(shakeCommand)) {
      commandScheduler.cancel(shakeCommand);
    }
  }

  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }
}
