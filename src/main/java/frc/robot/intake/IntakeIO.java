package frc.robot.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {}

    public static void updateInputs() {}

    public static void setBeltOpenLoop(double voltage) {}

    public static void setIntakeOpenLoop(double voltage) {}

    public static void setBeltStartOpenLoop(double voltage) {}
}
