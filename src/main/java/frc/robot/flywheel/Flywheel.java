package frc.robot.flywheel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheel extends SubsystemBase {
    private final FlywheelIO io;
    public Flywheel(FlywheelIO io) {
        this.io = io;
    }

    public void setVelocity(double speedRotationsPerSecond) {
        io.setVelocity(speedRotationsPerSecond);
    }

    public void stop() {
        io.setVelocity(0.0);
    }
}
