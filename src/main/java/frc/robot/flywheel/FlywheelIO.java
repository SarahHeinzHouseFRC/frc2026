package frc.robot.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
    @AutoLog
    public static class DriveIOInputs {
        public double velocityRotationsPerSecond = 0.0;
    }
    public default void updateInputs(DriveIOInputs inputs) {}
    public default void setVelocity(double speedRotationsPerSecond) {}
}
