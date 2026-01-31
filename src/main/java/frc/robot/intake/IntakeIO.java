package frc.robot.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    void setBeltStarOpenLoop(double voltage);

    @AutoLog
    public static class IntakeIOInputs {}

    default void updateInputs() {}

    default void setBeltOpenLoop(double voltage) {}

    default void setIntakeOpenLoop(double voltage) {}

    default void setBeltStartOpenLoop(double voltage) {}

    default void setOBIOpenLoop(double voltage) {}

    default public void setOBIPivotMotorOpenLoop(double voltage) {}

    default void setOBIPivotMotorClosedLoop(double voltage) {}
}
