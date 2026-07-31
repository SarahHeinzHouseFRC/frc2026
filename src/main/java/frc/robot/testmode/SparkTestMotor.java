package frc.robot.testmode;

import com.revrobotics.spark.SparkBase;

/** A named REV motor controller exposed to the test-mode fault monitor. */
public record SparkTestMotor(String name, SparkBase motor) implements TestMotor {
  @Override
  public int canId() {
    return motor.getDeviceId();
  }

  @Override
  public int activeFaultBits() {
    return motor.getFaults().rawBits;
  }

  @Override
  public int stickyFaultBits() {
    return motor.getStickyFaults().rawBits;
  }
}
