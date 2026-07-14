package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ShotCalculator;
import frc.robot.utils.Utils;

/**
 * Allows driver translation while holding the robot pointed along the calculated shot direction.
 */
public class DriveAutoShootCommand extends Command {
  private static final double TRANSLATION_SPEED_METERS_PER_SECOND = 3.5;
  private static final double HEADING_KP = 4.0;
  private static final double HEADING_TOLERANCE_RADIANS = 0.15;

  private final Drive drive;
  private final XboxController controller;
  private final ShotCalculator shotCalculator = ShotCalculator.getInstance();
  private final PIDController headingController = new PIDController(HEADING_KP, 0, 0);

  public DriveAutoShootCommand(XboxController controller, Drive drive) {
    this.drive = drive;
    this.controller = controller;

    headingController.enableContinuousInput(-Math.PI, Math.PI);
    headingController.setTolerance(HEADING_TOLERANCE_RADIANS);
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    headingController.reset();
    drive.setYawAtSetpoint(false);
  }

  @Override
  public void execute() {
    double vx =
        Utils.scaleAxis(
            Utils.deadband(-controller.getLeftY() * TRANSLATION_SPEED_METERS_PER_SECOND, .1), 2);
    double vy =
        Utils.scaleAxis(
            Utils.deadband(-controller.getLeftX() * TRANSLATION_SPEED_METERS_PER_SECOND, .1), 2);

    double currentHeading = drive.getPose().getRotation().getRadians();
    double targetHeading = currentHeading + shotCalculator.getShotAngle();
    double omega = headingController.calculate(currentHeading, targetHeading);
    drive.setYawAtSetpoint(headingController.atSetpoint());

    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            new ChassisSpeeds(vx, vy, omega), drive.getPose().getRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    drive.setYawAtSetpoint(false);
    drive.runVelocity(new ChassisSpeeds());
  }
}
