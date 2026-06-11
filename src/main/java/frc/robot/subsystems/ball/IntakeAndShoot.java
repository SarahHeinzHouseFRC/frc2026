package frc.robot.subsystems.ball;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.turret.Turret;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.ball.BallConstants.intakeSpeed;
import static frc.robot.subsystems.ball.BallConstants.presetEngaged;

public class IntakeAndShoot extends Command {
  private Ball ball;
  private DoubleSupplier intakeSpeedSupplier, flywheelSpeedSupplier;
  public IntakeAndShoot(Ball ball, DoubleSupplier intakeSpeedSupplier, DoubleSupplier flywheelSpeedSupplier) {
    this.ball = ball;
    this.intakeSpeedSupplier = intakeSpeedSupplier;
    this.flywheelSpeedSupplier = flywheelSpeedSupplier;
    addRequirements(ball);
  }

  @Override
  public void execute() {
    double intakeSpeedTarget = MathUtil.clamp(intakeSpeedSupplier.getAsDouble(), 0, 1);
    ball.setIntakePosition(presetEngaged);
    ball.runIntake(intakeSpeedTarget * intakeSpeed);
    ball.runFlywheel(flywheelSpeedSupplier.getAsDouble());
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
