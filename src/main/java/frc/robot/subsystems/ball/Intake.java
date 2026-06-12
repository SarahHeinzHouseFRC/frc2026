package frc.robot.subsystems.ball;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.ball.BallConstants.intakeSpeed;
import static frc.robot.subsystems.ball.BallConstants.presetEngaged;

public class Intake extends Command {
  private Ball ball;
  private DoubleSupplier speedSupplier;
  public Intake(Ball ball, DoubleSupplier speedSupplier) {
    this.ball = ball;
    this.speedSupplier = speedSupplier;
    addRequirements(ball);
  }

  @Override
  public void execute() {
    double speed = MathUtil.clamp(speedSupplier.getAsDouble(), -1, 1);
    ball.setIntakePosition(presetEngaged);
    ball.runIntake(speed * intakeSpeed);
    if (speed < 0) {
      ball.runIndexerAndBelt(speed);
    }
  }

  @Override
  public void end(boolean interrupted) {
    ball.stopIntake();
    ball.runIndexerAndBelt(0);
  }
}
