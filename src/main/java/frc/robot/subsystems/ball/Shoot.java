package frc.robot.subsystems.ball;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.launcher.LauncherSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.ball.BallConstants.*;

public class Shoot extends Command {
  private BallSubsystem ball;
  private LauncherSubsystem launcher;
  private RobotContainer container = RobotContainer.getInstance();
  private Drive drive = Drive.getInstance();
  private TurretSubsystem turret = TurretSubsystem.getInstance();
  private DoubleSupplier speedSupplier;

  public Shoot(BallSubsystem ball, LauncherSubsystem launcher, DoubleSupplier flywheelSpeedSupplier) {
    this.ball = ball;
    this.launcher = launcher;
    this.speedSupplier = flywheelSpeedSupplier;
    addRequirements(ball, launcher);
  }

  @Override
  public void execute() {
    double speed = speedSupplier.getAsDouble();
    ball.runIntake(intakeSpeedShooting);
    ball.runBelt(beltSpeedShooting);
    launcher.setFlywheelSetpoint(speed);
    if (TeleopBallControl.readyToShoot()) {
      ball.runIndexer(indexerSpeedShooting);
    } else {
      ball.runIndexer(-indexerSpeedPreShooting);
    }
  }

  @Override
  public void end(boolean interrupted) {
    ball.runIntake(0);
    ball.runIndexer(0);
    ball.runBelt(0);
    launcher.stopFlywheel();
  }
}
