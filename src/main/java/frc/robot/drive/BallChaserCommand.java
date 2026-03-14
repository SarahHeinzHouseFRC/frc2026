package frc.robot.drive;

import static frc.robot.Robot.VERSION;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import java.util.function.DoubleSupplier;

public class BallChaserCommand extends Command {
  private Drive drive;
  private DoubleSupplier steer;

  public BallChaserCommand(Drive drive, DoubleSupplier steer) {
    this.drive = drive;
    addRequirements(drive);
    this.steer = steer;
  }

  @Override
  public void initialize() {
    drive.stop();
  }

  @Override
  public void execute() {
    drive.runVelocity(
        ChassisSpeeds.fromRobotRelativeSpeeds(
            VERSION == Robot.RobotVersion.V1 ? -1 : -1,
            0,
            steer.getAsDouble() * (VERSION == Robot.RobotVersion.V2 ? 1 : 1),
            drive.getRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }
}
