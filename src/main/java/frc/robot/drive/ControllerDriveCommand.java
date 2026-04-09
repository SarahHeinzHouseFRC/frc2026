package frc.robot.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.shooter.Shooter;
import frc.robot.utils.Utils;

public class ControllerDriveCommand extends Command {
  private final Drive drive;
  private final XboxController controller;

  private static double lastTimeToldToGoSlow = 0;

  private double lastVx = 0;
  private double lastVy = 0;
  private double lastOmega = 0;

  private double dt = 1 / Robot.loopFrequency;

  public static void goSlowPls() {
    lastTimeToldToGoSlow = Timer.getTimestamp();
  }

  public ControllerDriveCommand(XboxController controller, Drive drive) {
    this.drive = drive;
    this.controller = controller;
    addRequirements(drive);
  }

  @Override
  public void execute() {
    double speed = 3.5;
    if (Shooter.getInstance().isShooting()) {
      speed = 1;
    }
    double angularSpeed = 3.5;
    double maxAccel = 14;
    double maxAlpha = 14;
    double vx = Utils.scaleAxis(Utils.deadband(-controller.getLeftY() * speed, .1), 2);
    double vy = Utils.scaleAxis(Utils.deadband(-controller.getLeftX() * speed, .1), 2);
    double omega = Utils.scaleAxis(Utils.deadband(-controller.getRightX() * angularSpeed, .1), 2);
    //    lastVx = Utils.clampStep(lastVx, vx, maxAccel * dt);
    //    lastVy = Utils.clampStep(lastVy, vy, maxAccel * dt);
    //    lastOmega = Utils.clampStep(lastOmega, omega, maxAlpha * dt);
    ChassisSpeeds speeds = new ChassisSpeeds(vx, vy, omega);
    drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drive.getRotation()));

    if (controller.getLeftBumperButton() && controller.getRightBumperButton()) {
      drive.zeroGyro();
    }
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }
}
