package frc.robot.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.Utils;

public class ControllerDriveCommand extends Command {
  private final Drive drive;
  private final XboxController controller;

  public ControllerDriveCommand(XboxController controller, Drive drive) {
    this.drive = drive;
    this.controller = controller;
    addRequirements(drive);
  }

  @Override
  public void execute() {
    double vx = Utils.scaleAxis(Utils.deadband(-controller.getLeftY() * 3.5, .1), 2);
    double vy = Utils.scaleAxis(Utils.deadband(-controller.getLeftX() * 3.5, .1), 2);
    double omega = Utils.scaleAxis(Utils.deadband(-controller.getRightX() * 3.5, .1), 2);
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
