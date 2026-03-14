package frc.robot.shooter;

import com.revrobotics.AbsoluteEncoder;
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

    public double position26Radians = 0.0;
    public double position28Radians = 0.0;
    public double velocity26RadiansPerSecond = 0.0;
    public double velocity28RadiansPerSecond = 0.0;

    public boolean encoder28Connected = false;
    public boolean encoder26Connected = false;
  }

  default void updateInputs(ShooterIOInputs inputs) {}

  default void setFlywheelVelocity(double speedRotationsPerSecond) {}

  default void setLinearActuatorPosition(double extensionMm) {}

  default void setTurretYaw(double yawRadians) {}

  default void setTurretPitchOpenLoop(double voltage) {}

  default void setTurretYawOpenLoop(double voltage) {}

  default void setFlywheelOpenLoop(double voltage) {}

  default void zeroYaw() {
    zeroYaw(0.0);
  }

  default void zeroYaw(double position) {}

  default void set28TurretAngleSupplier(AbsoluteEncoder enc) {}
}
