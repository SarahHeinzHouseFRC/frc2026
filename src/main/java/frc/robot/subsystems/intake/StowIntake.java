package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;

public class StowIntake extends Command {
  private IntakeSubsystem intake;
  public StowIntake(IntakeSubsystem intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.retract();
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {
  }
}
