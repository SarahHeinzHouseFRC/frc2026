package frc.robot.overbumper;

import frc.robot.Robot;

public class OverBumperConstants {
  public static final boolean tuningMode = false;
  public static final int overBumperMotorCanId = 25;
  public static final int overBumperPivotMotorCanId = 26;

  public static final double overBumperP;
  public static final double overBumperI;
  public static final double overBumperD;
  public static final double overBumperV;

  public static final double overBumperPivotP;
  public static final double overBumperPivotI;
  public static final double overBumperPivotD;

  // Value of the encoder when the intake is stowed away
  public static final double presetStowed;
  // Value of the encoder when the intake is in the engaged position
  public static final double presetEngaged;
  // How fast the intake should change position when using fine adjustment
  public static final double changeSpeed = 0.001;

  static {
    switch (Robot.VERSION) {
      case V1:
        overBumperP = 0.00013;
        overBumperI = 0;
        overBumperD = 0;
        overBumperV = 0.000195;
        overBumperPivotP = 6.0;
        overBumperPivotI = 0.0;
        overBumperPivotD = 0.0;
        presetStowed = -0.247;
        presetEngaged = -0.014;
        break;
      case V2:
        overBumperP = 0.001;
        overBumperI = 0;
        overBumperD = 0.00002;
        overBumperV = 0.00185;
        overBumperPivotP = 9.0;
        overBumperPivotI = 0.0;
        overBumperPivotD = 0.0;
        presetStowed = -0.247; // TODO: needs tuning
        presetEngaged = -0.10;
        break;
      default:
        throw new IllegalStateException("Invalid robot version");
    }
  }
}
