package frc.robot.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    void setBeltStarOpenLoop(double voltage);

    @AutoLog
    static class IntakeIOInputs {}

    default void updateInputs() {}

    default void setBeltOpenLoop(double voltage) {}

    default void setIntakeOpenLoop(double voltage) {}

    default void setBeltStartOpenLoop(double voltage) {}
}
