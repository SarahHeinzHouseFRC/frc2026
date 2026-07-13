package frc.robot.subsystems.ball;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class BallControl extends Command {
  private final BallSubsystem ball;
  private final Supplier<BallInputs> ballInputsSupplier;

  public BallControl(BallSubsystem ball, DoubleSupplier intakeSpeedSupplier, DoubleSupplier beltSpeedSupplier, DoubleSupplier indexerSpeedSupplier) {
    this(ball, () -> new BallInputs(intakeSpeedSupplier.getAsDouble(), beltSpeedSupplier.getAsDouble(), indexerSpeedSupplier.getAsDouble()));
  }

  public BallControl(BallSubsystem ball, Supplier<BallInputs> ballInputsSupplier) {
    this.ball = ball;
    this.ballInputsSupplier = ballInputsSupplier;
    addRequirements(ball);
  }

  @Override
  public void execute() {
    ball.runInputs(ballInputsSupplier.get());
  }

  @Override
  public void end(boolean interrupted) {
    ball.stop();
  }
}
