package frc.robot.testmode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.drive.Drive;

/** Exercises all four swerve directions and validates every module. */
public final class DriveTestCommand extends SubsystemTestCommand {
  private static final double TEST_SPEED_MPS = 2.0;
  private static final double SPEED_TOLERANCE_MPS = 0.65;
  private static final double TARGET_SPEED_TOLERANCE_MPS = 0.05;
  private static final double ANGLE_TOLERANCE_RADIANS = Math.toRadians(2.0);
  private static final String[] MODULE_NAMES = {
    "front-left", "front-right", "back-left", "back-right"
  };

  private double maxSpeedDeviation = 0.0;
  private double maxAngleDeviation = 0.0;

  private enum Phase {
    ORIENT,
    STOP_BEFORE_LEFT,
    LEFT,
    STOP_BEFORE_RIGHT,
    RIGHT,
    STOP_BEFORE_FORWARD,
    FORWARD,
    STOP_BEFORE_BACKWARD,
    BACKWARD
  }

  private final Drive drive;
  private Phase phase;

  public DriveTestCommand(Drive drive) {
    super("DRIVE", drive);
    this.drive = drive;
  }

  @Override
  protected void initializeTest() {
    stopTest();
    phase = Phase.ORIENT;
    maxSpeedDeviation = 0.0;
    maxAngleDeviation = 0.0;
    startPhase("orienting modules (faults ignored)");
    drive.runVelocity(new ChassisSpeeds(TEST_SPEED_MPS, 0.0, 0.0));
  }

  @Override
  protected void executeTest() {
    double elapsed = phaseElapsed();
    switch (phase) {
      case ORIENT -> {
        drive.runVelocity(new ChassisSpeeds(TEST_SPEED_MPS, 0.0, 0.0));
        if (elapsed >= 1.0) {
          stopTest();
          startFaultMonitoring(drive.getTestMotors());
          setPhase(Phase.STOP_BEFORE_LEFT, "stopped before left");
        }
      }
      case STOP_BEFORE_LEFT -> {
        if (elapsed >= 0.5) {
          setPhase(Phase.LEFT, "left (+y)");
        }
      }
      case LEFT ->
          runDirection(
              elapsed,
              "left",
              new ChassisSpeeds(0.0, TEST_SPEED_MPS, 0.0),
              Phase.STOP_BEFORE_RIGHT);
      case STOP_BEFORE_RIGHT -> {
        if (elapsed >= 0.5) {
          setPhase(Phase.RIGHT, "right (-y)");
        }
      }
      case RIGHT ->
          runDirection(
              elapsed,
              "right",
              new ChassisSpeeds(0.0, -TEST_SPEED_MPS, 0.0),
              Phase.STOP_BEFORE_FORWARD);
      case STOP_BEFORE_FORWARD -> {
        if (elapsed >= 0.5) {
          setPhase(Phase.FORWARD, "forward (+x)");
        }
      }
      case FORWARD ->
          runDirection(
              elapsed,
              "forward",
              new ChassisSpeeds(TEST_SPEED_MPS, 0.0, 0.0),
              Phase.STOP_BEFORE_BACKWARD);
      case STOP_BEFORE_BACKWARD -> {
        if (elapsed >= 0.5) {
          setPhase(Phase.BACKWARD, "backward (-x)");
        }
      }
      case BACKWARD ->
          runDirection(
              elapsed,
              "backward",
              new ChassisSpeeds(-TEST_SPEED_MPS, 0.0, 0.0),
              null);
    }
  }

  private void runDirection(
      double elapsed, String direction, ChassisSpeeds speeds, Phase nextStopPhase) {
    // Reapply the command as the modules turn so cosine scaling cannot leave the drive motors at
    // the reduced setpoint calculated from the steering angle at the start of the phase.
    drive.runVelocity(speeds);
    if (elapsed >= 0.5) {
      checkSetpoints(direction);
    }
    if (elapsed >= 1.0) {
      stopTest();
      if (nextStopPhase == null) {
        completeTest();
      } else {
        setPhase(nextStopPhase, "stopped after " + direction);
      }
    }
  }

  private void checkSetpoints(String direction) {
    SwerveModuleState[] actual = drive.getModuleStates();
    SwerveModuleState[] targets = drive.getTargetModuleStates();
    for (int i = 0; i < actual.length; i++) {
      if (!withinTolerance(
          Math.abs(targets[i].speedMetersPerSecond),
          TEST_SPEED_MPS,
          TARGET_SPEED_TOLERANCE_MPS)) {
        addFailure(direction + " " + MODULE_NAMES[i] + " target wheel speed outside ±0.05 m/s");
      }
      if (!withinTolerance(
          actual[i].speedMetersPerSecond,
          targets[i].speedMetersPerSecond,
          SPEED_TOLERANCE_MPS)) {
        addFailure(direction + " " + MODULE_NAMES[i] + " wheel speed outside ±0.65 m/s");
      }
      Rotation2d angleError = actual[i].angle.minus(targets[i].angle);
      if (Math.abs(angleError.getRadians()) > ANGLE_TOLERANCE_RADIANS) {
        addFailure(direction + " " + MODULE_NAMES[i] + " steering angle outside ±2 degrees");
      }

      double speedDeviation = Math.abs(actual[i].speedMetersPerSecond - targets[i].speedMetersPerSecond);
      double angleDeviation = Math.abs(angleError.getRadians());

      if (angleDeviation > maxAngleDeviation) maxAngleDeviation = angleDeviation;
      if (speedDeviation > maxSpeedDeviation) maxSpeedDeviation = speedDeviation;
    }
  }

  private void setPhase(Phase nextPhase, String status) {
    phase = nextPhase;
    startPhase(status);
  }

  @Override
  protected void stopTest() {
    setDebugString(String.format("%.2f/%.4f", maxSpeedDeviation, maxAngleDeviation));
    drive.runVelocity(new ChassisSpeeds());
  }
}
