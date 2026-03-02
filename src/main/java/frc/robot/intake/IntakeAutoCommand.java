package frc.robot.intake;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeAutoCommand extends Command {
  private final Intake intake;

  public IntakeAutoCommand(Intake intake) {
    addRequirements(intake);
    this.intake = intake;
  }

  @Override
  public void execute() {
    intake.autoShoot();
  }

  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }
}
