package frc.robot.overbumper;

import org.littletonrobotics.junction.AutoLog;

public interface OverBumperIO {
  @AutoLog
  class OverBumperIOInputs {
    public double obiPosition = 0.0;
    public double obiSpeed = 0.0;
  }

  default void updateInputs(OverBumperIOInputs inputs) {}

  default void setOBIClosedLoop(double speed) {}

  default void setOBIOpenLoop(double voltage) {}

  default void setOBIPivotMotorOpenLoop(double voltage) {}

  default void setOBIPivotMotorClosedLoop(double voltage) {}
}
