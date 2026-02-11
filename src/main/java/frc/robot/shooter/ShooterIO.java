package frc.robot.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  class ShooterIOInputs {
    public double turretYawRadians = 0.0;
    public double turretYawSetpointRadians = 0.0;
    public double turretPitchRadians = 0.0;
    public double flywheelVelocityRotationsPerMinute = 0.0;
    public double linearActuatorSetpointMm = 0.0;
    public double timestamp = 0.0;
  }

  default void updateInputs(ShooterIOInputs inputs) {}

  default void setFlywheelVelocity(double speedRotationsPerSecond) {}

  //  default void setTurretAngle(double yawRadians, double pitchRadians) {
  //    setTurretYaw(yawRadians);
  //    setTurretPitch(pitchRadians);
  //  }

  default void setLinearActuatorPosition(double extensionMm) {}

  default void setTurretYaw(double yawRadians) {}

  default void setTurretPitchOpenLoop(double voltage) {}

  default void setTurretYawOpenLoop(double voltage) {}

  default void setFlywheelOpenLoop(double voltage) {}

  default void zeroYaw() {}
}
