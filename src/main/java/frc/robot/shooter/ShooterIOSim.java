package frc.robot.shooter;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.simulator.Simulator;

public class ShooterIOSim implements ShooterIO {
  private double pitchRadians = 0.0;
  private double yawRadians = 0.0;
  private double flywheelVelocityRotationsPerSecond = 0.0;
  private double lastShootTime = 0.0;

  public void updateInputs(ShooterIOInputs inputs) {
    inputs.turretPitchRadians = pitchRadians;
    inputs.turretYawRadians = yawRadians;
    inputs.flywheelVelocityRotationsPerMinute = flywheelVelocityRotationsPerSecond;
    if (flywheelVelocityRotationsPerSecond > 0) {
      double currentTime = Timer.getFPGATimestamp();
      if (currentTime - lastShootTime > .1) {
        Simulator.getInstance()
            .shootBallFromRobot(pitchRadians, yawRadians, flywheelVelocityRotationsPerSecond);
        lastShootTime = currentTime;
      }
    }
  }

  public void setFlywheelVelocity(double speedRotationsPerSecond) {
    flywheelVelocityRotationsPerSecond = speedRotationsPerSecond;
  }

  public void setTurretAngle(double yawRadians, double pitchRadians) {
    setTurretYaw(yawRadians);
    setTurretPitch(pitchRadians);
  }

  public void setTurretPitch(double pitchRadians) {
    this.pitchRadians = pitchRadians;
  }

  public void setTurretYaw(double yawRadians) {
    this.yawRadians = yawRadians;
  }

  public void setTurretPitchOpenLoop(double voltage) {
    pitchRadians += voltage / 100;
  }

  public void setTurretYawOpenLoop(double voltage) {
    yawRadians += voltage / 100;
  }

  public void setFlywheelOpenLoop(double voltage) {
    flywheelVelocityRotationsPerSecond = voltage * 10;
  }
}
