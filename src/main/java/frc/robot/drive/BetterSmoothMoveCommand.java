package frc.robot.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.utils.Utils;

public class BetterSmoothMoveCommand extends Command {
  private final Drive drive = Drive.getInstance();
  private final Pose2d target;
  private double accelerationLimit = 5;
  private double velocityLimit = 5;

  private double headingP = 4; // TODO: experiment
  private double headingI = 0;
  private double headingD = 0;

  private double driveP = 4; // TODO: experiment
  private double driveI = 0;
  private double driveD = 0;

  private PIDController headingPidController = new PIDController(headingP, headingI, headingD);
  private PIDController xPidController = new PIDController(driveP, driveI, driveD);
  private PIDController yPidController = new PIDController(driveP, driveI, driveD);

  private final double dt = 1d / Robot.loopFrequency;
  private double positionTolerance = 0.1; // meters
  private double headingTolerance = .5;

  // Track current velocity for smooth acceleration
  private double currentVx = 0;
  private double currentVy = 0;

  public BetterSmoothMoveCommand(Pose2d target, boolean invertLeftRight) {
    this(invertLeftRight ? Utils.invertLeftRight(target) : target);
  }

  public BetterSmoothMoveCommand(Pose2d target) {
    this.target = target;
    headingPidController.enableContinuousInput(-Math.PI, Math.PI);
    xPidController.setTolerance(positionTolerance);
    yPidController.setTolerance(positionTolerance);
    headingPidController.setTolerance(headingTolerance);
    addRequirements(drive);
  }

  public BetterSmoothMoveCommand withAccelerationLimit(double accelerationLimit) {
    this.accelerationLimit = accelerationLimit;
    return this;
  }

  public BetterSmoothMoveCommand withVelocityLimit(double velocityLimit) {
    this.velocityLimit = velocityLimit;
    return this;
  }

  public BetterSmoothMoveCommand withPositionTolerance(double positionTolerance) {
    this.positionTolerance = positionTolerance;
    xPidController.setTolerance(positionTolerance);
    yPidController.setTolerance(positionTolerance);
    return this;
  }

  public BetterSmoothMoveCommand withHeadingTolerance(double headingTolerance) {
    this.headingTolerance = headingTolerance;
    headingPidController.setTolerance(headingTolerance);
    return this;
  }

  public BetterSmoothMoveCommand withPID(double p, double i, double d) {
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

    double desiredVx = xPidController.calculate(current.getX(), target.getX());
    double desiredVy = yPidController.calculate(current.getY(), target.getY());

    desiredVx = MathUtil.clamp(desiredVx, -velocityLimit, velocityLimit);
    desiredVy = MathUtil.clamp(desiredVy, -velocityLimit, velocityLimit);
    double maxDelta = accelerationLimit * dt;

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
    //    Translation2d current = drive.getPose().getTranslation();
    //    return Math.abs(current.getDistance(target.getTranslation())) < positionTolerance;
    //        && MathUtil.isNear(
    //            drive.getPose().getRotation().getRadians(),
    //            target.getRotation().getRadians(),
    //            headingTolerance);
    return xPidController.atSetpoint()
        && yPidController.atSetpoint()
        && headingPidController.atSetpoint();
  }

  /** Steps {@code current} toward {@code desired} by at most {@code maxDelta}. */
  private double clampStep(double current, double desired, double maxDelta) {
    double diff = desired - current;
    if (Math.abs(diff) <= maxDelta) return desired;
    return current + Math.copySign(maxDelta, diff);
  }
}
