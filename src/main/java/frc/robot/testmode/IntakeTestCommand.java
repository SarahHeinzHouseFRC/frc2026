package frc.robot.testmode;

import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeSubsystem;

/** Exercises and validates both intake pivot motors. */
public final class IntakeTestCommand extends SubsystemTestCommand {
  private static final double POSITION_TOLERANCE = 0.075;

  private double maxPositionDeviationStowed = 0.0;
  private double maxPositionDeviationDeployed = 0.0;

  private enum Phase {
    PREPARE_DEPLOYED,
    WAIT_BEFORE_TEST_START,
    STOW_MOVE,
    STOW_HOLD,
    DEPLOY_MOVE,
    DEPLOY_HOLD
  }

  private final IntakeSubsystem intake;
  private Phase phase;

  public IntakeTestCommand(IntakeSubsystem intake) {
    super("INTAKE", intake);
    this.intake = intake;
  }

  @Override
  protected void initializeTest() {
    stopTest();
    maxPositionDeviationStowed = 0.0;
    maxPositionDeviationDeployed = 0.0;
    startFaultMonitoring(intake.getTestMotors());
    intake.deploy();
    setPhase(Phase.PREPARE_DEPLOYED, "preparing deployed position");
  }

  @Override
  protected void executeTest() {
    double elapsed = phaseElapsed();
    switch (phase) {
      case PREPARE_DEPLOYED -> {
        if (atPosition(IntakeConstants.presetEngaged)) {
          setPhase(Phase.WAIT_BEFORE_TEST_START, "waiting");
        } else if (elapsed >= 2.0) {
          addFailure("initial deployment did not reach setpoint within 2 seconds");
          setPhase(Phase.WAIT_BEFORE_TEST_START, "waiting");
        }
      }
      case WAIT_BEFORE_TEST_START -> {
        if (elapsed >= 0.5) {
          setPhase(Phase.STOW_MOVE, "stowing");
        }
      }
      case STOW_MOVE -> {
        intake.retract();
        if (elapsed >= 1.0) {
          setPhase(Phase.STOW_HOLD, "verifying stowed");
        }
      }
      case STOW_HOLD -> {
        intake.retract();
        checkPosition("stowed", IntakeConstants.presetStowed);
        if (elapsed >= 1.0) {
          setPhase(Phase.DEPLOY_MOVE, "deploying");
        }
      }
      case DEPLOY_MOVE -> {
        intake.deploy();
        if (elapsed >= 1.0) {
          setPhase(Phase.DEPLOY_HOLD, "verifying deployed");
        }
      }
      case DEPLOY_HOLD -> {
        intake.deploy();
        checkPosition("deployed", IntakeConstants.presetEngaged);
        if (elapsed >= 1.0) {
          stopTest();
          completeTest();
        }
      }
    }
  }

  private boolean atPosition(double target) {
    for (double position : intake.getPivotPositions()) {
      if (!withinTolerance(position, target, POSITION_TOLERANCE)) {
        return false;
      }
    }
    return true;
  }

  private void checkPosition(String state, double target) {
    double[] positions = intake.getPivotPositions();
    for (int i = 0; i < positions.length; i++) {
      if (!withinTolerance(positions[i], target, POSITION_TOLERANCE)) {
        addFailure("pivot " + (i + 1) + " did not remain " + state);
      }

      double deviation = Math.abs(positions[i] - target);
      switch (state) {
        case "stowed" -> {
          if (deviation > maxPositionDeviationStowed) maxPositionDeviationStowed = deviation;
        }
        case "deployed" -> {
          if (deviation > maxPositionDeviationDeployed) maxPositionDeviationDeployed = deviation;
        }
      }
    }
  }

  private void setPhase(Phase nextPhase, String status) {
    phase = nextPhase;
    startPhase(status);
  }

  @Override
  protected void stopTest() {
    setDebugString(String.format("%.03f/%.03f", maxPositionDeviationStowed, maxPositionDeviationDeployed));
    intake.stop();
  }
}
