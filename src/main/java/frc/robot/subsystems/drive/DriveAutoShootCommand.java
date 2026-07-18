package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ShotCalculator;
import frc.robot.utils.Utils;

import java.util.function.DoubleSupplier;

/**
 * Allows driver translation while holding the robot pointed along the calculated shot direction.
 */
public class DriveAutoShootCommand extends Command {
  private static final double TRANSLATION_SPEED_METERS_PER_SECOND = 3.5;
  private static final double BOOST_TRANSLATION_SPEED_METERS_PER_SECOND = 5.0;
  private static final double HEADING_KP = 8.0;
  private static final double MAX_OMEGA = 3.0;
  private static final double HEADING_TOLERANCE_RADIANS = 0.10;

  private final Drive drive;
  private final DoubleSupplier vxSupplier;
  private final DoubleSupplier vySupplier;
  private final ShotCalculator shotCalculator = ShotCalculator.getInstance();
  private final PIDController headingController = new PIDController(HEADING_KP, 0, 0);

  public DriveAutoShootCommand(XboxController controller, Drive drive) {
    this(
        () -> Utils.scaleAxis(Utils.deadband(-controller.getLeftY() * (controller.getLeftStickButton() ? BOOST_TRANSLATION_SPEED_METERS_PER_SECOND : TRANSLATION_SPEED_METERS_PER_SECOND), .1), 2),
        () -> Utils.scaleAxis(Utils.deadband(-controller.getLeftX() * (controller.getLeftStickButton() ? BOOST_TRANSLATION_SPEED_METERS_PER_SECOND : TRANSLATION_SPEED_METERS_PER_SECOND), .1), 2),
        drive
    );
  }

  public DriveAutoShootCommand(Drive drive) {
    this(() -> 0.0, () -> 0.0, drive);
  }

  public DriveAutoShootCommand(DoubleSupplier vxSupplier, DoubleSupplier vySupplier, Drive drive) {
    this.vxSupplier = vxSupplier;
    this.vySupplier = vySupplier;
    this.drive = drive;
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
    double vx = vxSupplier.getAsDouble();
    double vy = vySupplier.getAsDouble();

    double currentHeading = drive.getPose().getRotation().getRadians();
    double targetHeading = currentHeading + shotCalculator.getShotAngle();
    double omega = MathUtil.clamp(headingController.calculate(currentHeading, targetHeading), -MAX_OMEGA, MAX_OMEGA);
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
