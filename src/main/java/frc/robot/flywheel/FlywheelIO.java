package frc.robot.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
    @AutoLog
    public static class FlywheelIOInputs {
        public double velocityRotationsPerSecond = 0.0;
    }
    public default void updateInputs(FlywheelIOInputs inputs) {}
    public default void setVelocity(double speedRotationsPerSecond) {}
}
