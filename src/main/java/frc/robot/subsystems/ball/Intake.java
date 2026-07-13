package frc.robot.subsystems.ball;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.ball.BallConstants.*;

public class Intake extends Command {
  private final BallSubsystem ball;
  private final IntakeSubsystem intake;
  private final DoubleSupplier speedSupplier;
  public Intake(BallSubsystem ball, IntakeSubsystem intake, DoubleSupplier speedSupplier) {
    this.ball = ball;
    this.intake = intake;
    this.speedSupplier = speedSupplier;
    addRequirements(ball, intake);
  }

  @Override
  public void execute() {
    double speed = MathUtil.clamp(speedSupplier.getAsDouble(), -1, 1);
    intake.deploy();
    ball.runIntake(speed * intakeSpeedIntaking);
    ball.runBelt(speed * beltSpeedIntaking);
    ball.runIndexer(Math.abs(speed) * indexerSpeedIntaking);
  }

  @Override
  public void end(boolean interrupted) {
    ball.runIntake(0);
    ball.runBelt(0);
    ball.runIndexer(0);
  }
}
