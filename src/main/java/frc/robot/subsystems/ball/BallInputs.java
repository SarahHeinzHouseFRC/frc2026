package frc.robot.subsystems.ball;

public record BallInputs(double intakeSpeed, double beltSpeed, double indexerSpeed) {
  public static final BallInputs ZERO = new BallInputs(0, 0, 0);
}