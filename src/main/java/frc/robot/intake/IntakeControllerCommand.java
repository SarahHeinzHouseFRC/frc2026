package frc.robot.intake;

import static frc.robot.intake.IntakeConstants.*;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

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
    //    System.out.println("intaking: " + intaking + ", shooting: " + shooting + ", outtaking:" +
    // outtaking);
    // if shooting or intaking
    double intakeMotor = intaking ? -1 : 0;

    // outtaking
    if (intakeMotor == 0 && outtaking) {
      intakeMotor = 1;
    }

    // if shooting and intaking
    double indexerMotor = 0;

    if (shooting && intaking) {
      indexerMotor = -.5;
    }

    if (shooting && !intaking) {
      indexerMotor = -1;
    }
    // if not shooting but intaking
    if (!shooting && intaking) {
      indexerMotor = 1;
    }
    if (indexerMotor == 0 && outtaking) {
      indexerMotor = -1;
    }
    // belt motor enabled if intaking but not shooting (in) +
    // belt motor enabled if shooting but not intaking (out) -
    double beltMotor = 0;
    if (!intaking && !shooting && outtaking) {
      beltMotor = 1;
    }
    if (!intaking && shooting) {
      beltMotor = 1;
    }
    if (intaking && !shooting) {
      beltMotor = -1;
    }
    if (intaking && shooting) {
      beltMotor = -1;
    }

    intake.setIntakeOpenLoop(intakeMotor * 1);
    intake.setBeltStarOpenLoop(indexerMotor * 1);
    intake.setBeltOpenLoop(beltMotor * 1);
    intake.setAgitatorOpenLoop(beltMotor * .5);
    //    System.out.println("intake: " + intakeMotor + ", index: " + indexerMotor + ", belt:" +
    // beltMotor);

    if (driverController.getLeftTriggerAxis() > .1) {
      switch (Robot.VERSION) {
        case V1:
          intake.setOBIClosedLoop(driverController.getLeftTriggerAxis() * 1600);
        case V2:
          intake.setOBIClosedLoop(
              driverController.getLeftTriggerAxis()
                  * 2250
                  * (driverController.getLeftBumperButton() ? -1 : 1));
      }

    } else {
      intake.setOBIOpenLoop(0);
    }

    if (driverController.getPOV() == 0) intake.setObiSetpoint(presetStowed);
    if (driverController.getPOV() == 90) intake.editObiSetpoint(-changeSpeed);
    if (driverController.getPOV() == 180) intake.setObiSetpoint(presetEngaged);
    if (driverController.getPOV() == 270) intake.editObiSetpoint(changeSpeed);
  }
}
