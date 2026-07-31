package frc.robot.testmode;

import frc.robot.subsystems.ball.BallSubsystem;

/** Runs and validates all three ball-path motors in both directions. */
public final class BallTestCommand extends SubsystemTestCommand {
  private static final double[] MINIMUM_RPMS = {4500.0, 4500.0, 4500.0};
  private static final String[] MOTOR_NAMES = {"intake", "belt", "indexer"};

  private double[] minAbsoluteRpms = {9999.0, 9999.0, 9999.0};

  private enum Phase {
    FORWARD,
    REVERSE
  }

  private final BallSubsystem ball;
  private Phase phase;

  public BallTestCommand(BallSubsystem ball) {
    super("BALL", ball);
    this.ball = ball;
  }

  @Override
  protected void initializeTest() {
    stopTest();
    minAbsoluteRpms = new double[]{9999.0, 9999.0, 9999.0};
    startFaultMonitoring(ball.getTestMotors());
    runAll(1.0);
    setPhase(Phase.FORWARD, "full forward");
  }

  @Override
  protected void executeTest() {
    double elapsed = phaseElapsed();
    if (elapsed >= 1.0) {
      checkVelocities();
    }
    if (elapsed >= 2.0) {
      if (phase == Phase.FORWARD) {
        runAll(-1.0);
        setPhase(Phase.REVERSE, "full reverse");
      } else {
        stopTest();
        completeTest();
      }
    }
  }

  private void runAll(double speed) {
    ball.runIntake(speed);
    ball.runBelt(speed);
    ball.runIndexer(speed);
  }

  private void checkVelocities() {
    double[] velocities = ball.getMotorVelocities();
    for (int i = 0; i < velocities.length; i++) {
      double absoluteVelocity = Math.abs(velocities[i]);

      if (absoluteVelocity < minAbsoluteRpms[i]) {
        minAbsoluteRpms[i] = absoluteVelocity;
      }

      boolean atExpectedVelocity =
          phase == Phase.FORWARD
              ? velocities[i] >= MINIMUM_RPMS[i]
              : velocities[i] <= -MINIMUM_RPMS[i];
      if (!atExpectedVelocity) {
        addFailure(
            MOTOR_NAMES[i]
                + (phase == Phase.FORWARD
                    ? " below +"  + MINIMUM_RPMS[i] + " RPM while running forward"
                    : " above -" + MINIMUM_RPMS[i] + " RPM while running reverse"));
      }
    }
  }

  private void setPhase(Phase nextPhase, String status) {
    phase = nextPhase;
    startPhase(status);
  }

  @Override
  protected void stopTest() {
    setDebugString(String.format("intake %.2f, belt %.2f, indexer %.2f", minAbsoluteRpms[0], minAbsoluteRpms[1], minAbsoluteRpms[2]));
    ball.stop();
  }
}
