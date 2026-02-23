package frc.robot.flywheel;

import edu.wpi.first.wpilibj2.command.Command;

public class FlywheelVelocityCommand extends Command {

  private final Flywheel flywheel;
  private final double targetVelocity; // in RPM or units your encoder uses

  public FlywheelVelocityCommand(Flywheel flywheel, double targetVelocity) {
    this.flywheel = flywheel;
    this.targetVelocity = targetVelocity;
    addRequirements(flywheel);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // Continuously command the flywheel to maintain velocity
    flywheel.setVelocity(targetVelocity);
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the flywheel when command ends
    flywheel.stop();
  }

  @Override
  public boolean isFinished() {
    // Run indefinitely until canceled
    return false;
  }
}
