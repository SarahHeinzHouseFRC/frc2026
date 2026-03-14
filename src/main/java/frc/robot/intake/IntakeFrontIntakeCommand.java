package frc.robot.intake;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeFrontIntakeCommand extends Command {
    private Intake intake;
    public IntakeFrontIntakeCommand(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }
    @Override
    public void initialize() {
        intake.stop();
    }
    @Override
    public void execute() {
        intake.intake();
    }
    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}
