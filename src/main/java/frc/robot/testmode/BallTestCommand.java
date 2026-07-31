package frc.robot.testmode;

import frc.robot.subsystems.ball.BallSubsystem;

/** Runs and validates all three ball-path motors in both directions. */
public final class BallTestCommand extends SubsystemTestCommand {
  private static final double MINIMUM_RPM = 1000.0;
  private static final String[] MOTOR_NAMES = {"intake", "belt", "indexer"};

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
    startFaultMonitoring(ball.getTestMotors());
    runAll(1.0);
    setPhase(Phase.FORWARD, "full forward");
  }

  @Override
  protected void executeTest() {
    double elapsed = phaseElapsed();
    if (elapsed >= 1.5) {
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
      boolean atExpectedVelocity =
          phase == Phase.FORWARD
              ? velocities[i] >= MINIMUM_RPM
              : velocities[i] <= -MINIMUM_RPM;
      if (!atExpectedVelocity) {
        addFailure(
            MOTOR_NAMES[i]
                + (phase == Phase.FORWARD
                    ? " below +3000 RPM while running forward"
                    : " above -3000 RPM while running reverse"));
      }
    }
  }

  private void setPhase(Phase nextPhase, String status) {
    phase = nextPhase;
    startPhase(status);
  }

  @Override
  protected void stopTest() {
    ball.stop();
  }
}
