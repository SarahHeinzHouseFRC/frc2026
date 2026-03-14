package frc.robot.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class SmoothMoveCommand extends Command {
  private final Drive drive = Drive.getInstance();
  private final Pose2d target;
  private double accelerationLimit = 5;
  private double velocityLimit = 5;

  private double headingP = 1;
  private double headingI = 0;
  private double headingD = 0;

  private PIDController headingPidController = new PIDController(headingP, headingI, headingD);

  private final double dt = 1d / Robot.loopFrequency;
  private static final double POSITION_TOLERANCE = 0.05; // meters

  // Track current velocity for smooth acceleration
  private double currentVx = 0;
  private double currentVy = 0;

  public SmoothMoveCommand(Pose2d target) {
    this.target = target;
    headingPidController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(drive);
  }

  public SmoothMoveCommand withAccelerationLimit(double accelerationLimit) {
    this.accelerationLimit = accelerationLimit;
    return this;
  }

  public SmoothMoveCommand withVelocityLimit(double velocityLimit) {
    this.velocityLimit = velocityLimit;
    return this;
  }

  public SmoothMoveCommand withPID(double p, double i, double d) {
    headingP = p;
    headingI = i;
    headingD = d;
    headingPidController.setPID(p, i, d);
    return this;
  }

  @Override
  public void initialize() {
    currentVx = 0;
    currentVy = 0;
  }

  @Override
  public void execute() {
    Translation2d current = drive.getPose().getTranslation();
    Translation2d error = target.getTranslation().minus(current);
    double distance = error.getNorm();

    if (distance < 1e-6) {
      drive.stop();
      return;
    }

    // Direction unit vector toward target
    double dirX = error.getX() / distance;
    double dirY = error.getY() / distance;

    // Desired speed: ramp down as we approach the target to avoid overshoot.
    // The ramp distance is the stopping distance at current speed given the acceleration limit:
    //   d_stop = v^2 / (2 * a)
    // We want v such that v^2 / (2*a) <= distance, i.e. v <= sqrt(2*a*distance)
    double rampedSpeed = Math.sqrt(2.0 * accelerationLimit * distance);
    double desiredSpeed = Math.min(velocityLimit, rampedSpeed);


    double desiredVx = dirX * desiredSpeed;
    double desiredVy = dirY * desiredSpeed;

    // Clamp acceleration step
    double maxDelta = accelerationLimit * dt;
    if (rampedSpeed < velocityLimit) {//deceleration
      maxDelta *= 1.5;
    }
    currentVx = clampStep(currentVx, desiredVx, maxDelta);
    currentVy = clampStep(currentVy, desiredVy, maxDelta);

    double omega =
        headingPidController.calculate(
            drive.getPose().getRotation().getRadians(), target.getRotation().getRadians());

    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            new ChassisSpeeds(currentVx, currentVy, omega), drive.getRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  @Override
  public boolean isFinished() {
    Translation2d current = drive.getPose().getTranslation();
    return current.getDistance(target.getTranslation()) < POSITION_TOLERANCE;
  }

  /** Steps {@code current} toward {@code desired} by at most {@code maxDelta}. */
  private double clampStep(double current, double desired, double maxDelta) {
    double diff = desired - current;
    if (Math.abs(diff) <= maxDelta) return desired;
    return current + Math.copySign(maxDelta, diff);
  }
}
