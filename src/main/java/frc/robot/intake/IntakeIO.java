package frc.robot.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  class IntakeIOInputs {}

  default void updateInputs(IntakeIOInputs inputs) {}

  default void setBeltOpenLoop(double voltage) {}

  default void setIntakeOpenLoop(double voltage) {}

  default void setIndexerOpenLoop(double voltage) {}
}
