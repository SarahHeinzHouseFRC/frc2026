package frc.robot.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.GenericController;
import frc.robot.Utils;
import frc.robot.commands.Command;

public class ControllerDriveCommand extends Command {
  private final Drive drive;
  private final GenericController controller;

  public ControllerDriveCommand(GenericController controller, Drive drive) {
    this.drive = drive;
    this.controller = controller;
    addRequirements(drive);
  }

  @Override
  public void execute() {
    double vx = Utils.scaleAxis(Utils.deadband(controller.readAnalog(3) * 1, .1), 2);
    double vy = Utils.scaleAxis(Utils.deadband(controller.readAnalog(2) * 1, .1), 2);
    double omega = Utils.scaleAxis(Utils.deadband(controller.readAnalog(4) * 1, .1), 2);
    ChassisSpeeds speeds = new ChassisSpeeds(vx, vy, omega);
    drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drive.getRotation()));

    if (controller.readDigital(0) && controller.readDigital(1)) {
      drive.zeroGyro();
    }
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }
}
