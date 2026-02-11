package frc.robot.shooter;

public record ShotParams(
    double flywheelVelocityRotationsPerMinute,
    double linearActuatorExtensionMillimeters,
    double yawOffsetRadians) {}
