package frc.robot.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double obiPosition = 0.0;
    public double obiSpeed = 0.0;
  }

  default void updateInputs(IntakeIOInputs inputs) {}

  default void setBeltOpenLoop(double voltage) {}

  default void setIntakeOpenLoop(double voltage) {}

  default void setBeltStarOpenLoop(double voltage) {}

  default void setOBIClosedLoop(double speed) {}

  default void setOBIOpenLoop(double voltage) {}

  default void setOBIPivotMotorOpenLoop(double voltage) {}

  default void setOBIPivotMotorClosedLoop(double voltage) {}
}
