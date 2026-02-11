package frc.robot.shooter;

public class ShotCalculators {

  private ShotCalculators() {}

  // starts at 2m center to center, increment .5m
  // rpm, mm, rpm, mm, etc.
  private static float[] lut =
      new float[] {
        2950f, 15f,
        3200f, 25f,
        3300f, 30f,
        3500f, 35f,
        3600f, 45f,
        3800f, 45f,
        3950f, 55f,
        4150f, 60f
      };

  public static final ShotCalculator lutShotCalculator =
      (distanceMeters, velocityRadialMetersPerSecond, velocityTangentialMetersPerSecond) -> {
        int n = (int) ((distanceMeters - 2) * 2);
        if (n < 0) {
          n = 0;
        } else if (n >= 6) {
          n = 6;
        }
        int idx = n * 2;
        double rpmLow = lut[idx];
        double linearLow = lut[idx + 1];
        double rpmHigh = lut[idx + 2];
        double linearHigh = lut[idx + 3];
        double rpmRange = rpmHigh - rpmLow;
        double linearRange = linearHigh - linearLow;
        double distanceLow = (double) n / 2 + 2;
        double distanceRange = 0.5;
        double t = (distanceMeters - distanceLow) / distanceRange;
        double lerpRpm = rpmLow + t * rpmRange;
        double lerpLinear = linearLow + t * linearRange;
        return new ShotParams(lerpRpm, lerpLinear, 0);
      };

  private static double pitchFromLinearActuator(double linearActuatorExtensionMillimeters) {
    return 0;
  }

  private static double linearActuatorFromPitch(double pitchRadians) {
    return 0;
  }

  private static final double flywheelEfficiencyRatio = 0.5;

  private static double velocityFromFlywheel(double flywheelVelocityRotationsPerMinute) {
    return flywheelVelocityRotationsPerMinute
        / 60 /* -> rotations per second */
        * (2 * Math.PI) /* -> radians per second */
        * 0.0508 /* radius in meters -> meters per second */
        * flywheelEfficiencyRatio;
  }

  private static double flywheelFromVelocity(double velocityMetersPerSecond) {
    return velocityMetersPerSecond
        / flywheelEfficiencyRatio
        / 0.0508 /* radius in meters -> radians per second */
        / (2 * Math.PI) /* radians per second -> rotations per second */
        * 60 /* -> rotations per minute */;
  }
}
