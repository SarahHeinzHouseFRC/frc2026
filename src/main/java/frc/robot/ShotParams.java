package frc.robot;

public record ShotParams(
    double flywheelVelocityRotationsPerMinute,
    double linearActuatorExtensionMillimeters,
    double yawOffsetRadians) {}
