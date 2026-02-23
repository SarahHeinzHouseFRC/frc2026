package frc.robot.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {}

  default void updateInputs(ClimberIOInputs inputs) {}

  default void runClimberOpenLoop(double voltage) {}
}
