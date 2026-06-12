package frc.robot.subsystems.ball;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.ball.BallConstants.*;

public class StowIntake extends Command {
  private Ball ball;
  public StowIntake(Ball ball) {
    this.ball = ball;
    addRequirements(ball);
  }

  @Override
  public void initialize() {
    ball.setIntakePositionSlowly(presetStowed);
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {
  }
}
