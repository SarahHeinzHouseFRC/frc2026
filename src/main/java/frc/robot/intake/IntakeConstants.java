package frc.robot.intake;

import frc.robot.Robot;

public class IntakeConstants {
  public static final int intakeMotorCanId = 20;
  public static final int beltStarMotorCanId = 22;
  public static final int beltMotorCanId = 23;
  public static final int overBumperMotorCanId = 25;
  public static final int overBumperPivotMotorCanId = 26;
  public static final int agitatorMotorCanId = 27;
  public static final double overBumperP;
  public static final double overBumperI;
  public static final double overBumperD;
  public static final double overBumperV;
  public static final double overBumperPivotP;
  public static final double overBumperPivotI;
  public static final double overBumperPivotD;

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
        break;
      case V2:
        overBumperP = 0.0001;
        overBumperI = 0.0;
        overBumperD = 0.0;
        overBumperV = 0.0002;
        overBumperPivotP = 6.0;
        overBumperPivotI = 0.0;
        overBumperPivotD = 0.0;
        break;
      default:
        throw new IllegalStateException("Invalid robot version");
    }
  }
}
