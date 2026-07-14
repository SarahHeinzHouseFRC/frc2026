package frc.robot.subsystems.drive;

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
    double speed = 3.5;
    if (controller.getLeftStickButton()) speed = 5.0;
    double angularSpeed = 3.5;
    if (controller.getRightStickButton()) angularSpeed = 5.0;
    double vx = Utils.scaleAxis(Utils.deadband(-controller.getLeftY() * speed, .1), 2);
    double vy = Utils.scaleAxis(Utils.deadband(-controller.getLeftX() * speed, .1), 2);
    double omega = Utils.scaleAxis(Utils.deadband(-controller.getRightX() * angularSpeed, .1), 2);

    ChassisSpeeds speeds = new ChassisSpeeds(vx, vy, omega);
    drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drive.getPose().getRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    drive.runVelocity(new ChassisSpeeds());
  }
}
