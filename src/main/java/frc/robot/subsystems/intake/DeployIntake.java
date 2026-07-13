package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;

public class DeployIntake extends Command {
  private IntakeSubsystem intake;
  public DeployIntake(IntakeSubsystem intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.deploy();
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {
  }
}
