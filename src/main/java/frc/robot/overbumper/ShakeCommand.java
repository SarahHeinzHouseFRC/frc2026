package frc.robot.overbumper;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class ShakeCommand extends Command {
  private double amplitude = 0.03;
  private double period = .5;
  private double startTime = 0.015;
  private double middle = -.1;
  private final OverBumper overBumper;

  public ShakeCommand(OverBumper overBumper) {
    this.overBumper = overBumper;
    addRequirements(overBumper);
  }

  public ShakeCommand(double amplitude, double period, double middle, OverBumper overBumper) {
    this.amplitude = amplitude;
    this.period = period;
    this.overBumper = overBumper;
    this.middle = middle;
    addRequirements(overBumper);
  }

  @Override
  public void initialize() {
    startTime = Timer.getTimestamp();
    overBumper.stopIntake();
    overBumper.stopPivot();
  }

  @Override
  public void execute() {
    double target =
        middle + Math.sin((Timer.getTimestamp() - startTime) * 2 * Math.PI / period) * amplitude;
    overBumper.setObiSetpoint(target);
    overBumper.setOBIClosedLoop(2000);
  }

  @Override
  public void end(boolean interrupted) {
    overBumper.stopPivot();
    overBumper.stopIntake();
  }
}
