package frc.robot.intake;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.Command;

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
        // if shooting or intaking
        double intakeMotor = intaking || shooting ? -1 : 0;

        //outtaking
        if (intakeMotor == 0 && outtaking) {
            intakeMotor = 1;
        }

        // if shooting
        double indexerMotor = shooting ? -1 : 0;

        // if not shooting but intaking
        if (indexerMotor == 0 && intaking) {
            indexerMotor = 1;
        }
        // belt motor enabled if intaking but not shooting (in) +
        // belt motor enabled if shooting but not intaking (out) -
        double beltMotor = 0;
        if (intaking && !shooting) {
            beltMotor = -1;
        }

        if (!intaking && shooting) {
            beltMotor = 1;
        }

        intake.setIntakeOpenLoop(intakeMotor * .5);
        intake.setBeltStarOpenLoop(indexerMotor * .5);
        intake.setBeltOpenLoop(beltMotor * .5);

        intake.setOBIOpenLoop(driverController.getLeftTriggerAxis() * 4);

        double obipivotol = 0;

        if (driverController.getPOV() == 0) {
            obipivotol = 1;
        }

        if (driverController.getPOV() == 180) {
            obipivotol = -1;
        }

//        intake.io.setOBIPivotMotorOpenLoop(obipivotol-.3);
        if (driverController.getPOV() == 0) intake.io.setOBIPivotMotorClosedLoop(intake.io.getOBIPivotMotorSetpoint() + 0.1d);
        if (driverController.getPOV() == 180) intake.io.setOBIPivotMotorClosedLoop(intake.io.getOBIPivotMotorSetpoint() - 0.1d);
    }
}
