package frc.robot.shooter;

import frc.robot.Robot;

public class ShooterConstants {
  public static final boolean tuningMode = false;
  public static final double motorToFlywheel;
  public static final double flywheelToMotor;
  public static final double flywheelP;
  public static final double flywheelI;
  public static final double flywheelD;
  public static final double flywheelV;
  public static final double yawReduction;
  public static final double yawP;
  public static final double yawI;
  public static final double yawD;
  public static final double maxYawVolts;
  public static final double yawMax;
  public static final double yawMin;
  public static final double yawModuloMin;
  public static final double yawModuloMax;

  static {
    switch (Robot.VERSION) {
      case V1:
        motorToFlywheel = 13d / 9;
        flywheelToMotor = 9d / 13;
        flywheelP = 0.0003;
        flywheelI = 0.0;
        flywheelD = 0.0;
        flywheelV = 0.000043;
        yawP = 3;
        yawI = 0;
        yawD = 0;
        yawReduction = 30d * (200d / 28d);
        maxYawVolts = 6;
        yawMax = Math.PI / 2;
        yawMin = -Math.PI / 2;
        yawModuloMax = Math.PI;
        yawModuloMin = -Math.PI;
        break;
      case V2:
        motorToFlywheel = 1.0;
        flywheelToMotor = 1.0;
        flywheelP = 0.00025;
        flywheelI = 0.0;
        flywheelD = 0.0;
        flywheelV = 0.00015;
        yawP = 2.5;
        yawI = 0;
        yawD = .025;
        yawReduction = 20d * (200d / 26d);
        maxYawVolts = 6;
        yawMax = Math.PI / 2;
        yawMin = -5 * Math.PI / 4;
        yawModuloMax = Math.PI / 2;
        yawModuloMin = -3 * Math.PI / 2;
        break;
      default:
        throw new IllegalStateException("Invalid robot version");
    }
  }
}
