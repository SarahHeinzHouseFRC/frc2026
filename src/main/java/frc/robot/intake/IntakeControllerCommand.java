package frc.robot.intake;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeControllerCommand extends Command {
  private final Intake intake;
  private final XboxController driverController;
  private final XboxController operatorController;

  public IntakeControllerCommand(XboxController driver, XboxController operator, Intake intake) {
    addRequirements(intake);
    this.intake = intake;
    this.driverController = driver;
    this.operatorController = operator;
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
  }

  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }
}
