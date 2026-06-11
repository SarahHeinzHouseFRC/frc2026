package frc.robot.subsystems.ball;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.turret.Turret;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.ball.BallConstants.intakeSpeed;
import static frc.robot.subsystems.ball.BallConstants.presetEngaged;

public class Shoot extends Command {
  private Ball ball;
  private DoubleSupplier speedSupplier;
  public Shoot(Ball ball, DoubleSupplier flywheelSpeedSupplier) {
    this.ball = ball;
    this.speedSupplier = flywheelSpeedSupplier;
    addRequirements(ball);
  }

  @Override
  public void execute() {
    double speed = speedSupplier.getAsDouble();
    ball.setIntakePositionSlowly(presetEngaged);
    ball.runIntake(intakeSpeed * 0.5);
    ball.runFlywheel(speed);
    if (ball.isFlywheelAtSpeed() && Turret.getInstance().isPanAtSetpoint()) {
      ball.runIndexerAndBelt(1);
    }
  }

  @Override
  public void end(boolean interrupted) {
    ball.runIntake(0);
    ball.runIndexerAndBelt(0);
    ball.stopFlywheel();
  }
}
