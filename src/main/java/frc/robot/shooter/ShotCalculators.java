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

        // xy plane is field
        // z is up
        double velocityY =
            velocityFromFlywheel(lerpRpm) * Math.cos(pitchFromLinearActuator(lerpLinear));
        double velocityZ =
            velocityFromFlywheel(lerpRpm) * Math.sin(pitchFromLinearActuator(lerpLinear));
        double velocityX = 0;
        velocityY += velocityRadialMetersPerSecond;
        velocityX -= velocityTangentialMetersPerSecond;
        double yawOffsetRadians = -Math.atan2(velocityX, velocityY);
        double angle =
            Math.atan2(velocityZ, Math.sqrt(velocityX * velocityX + velocityY * velocityY));
        return new ShotParams(
            flywheelFromVelocity(
                Math.sqrt(velocityX * velocityX + velocityY * velocityY + velocityZ * velocityZ)),
            linearActuatorFromPitch(angle),
            yawOffsetRadians);
      };

  private static double pitchFromLinearActuator(double linearActuatorExtensionMillimeters) {
    double linearActuatorLength = 168 + linearActuatorExtensionMillimeters;
    double linearActuatorTopToPivot = 159;
    double linearActuatorBottomToPivot = 251;
    double angleOffsetRadians = -0.35;
    // A = arccos((b^2 + c^2 - a^2)/(2bc))
    double triangleAngle =
        Math.acos(
            (linearActuatorTopToPivot * linearActuatorTopToPivot
                    + linearActuatorBottomToPivot * linearActuatorBottomToPivot
                    - linearActuatorLength * linearActuatorLength)
                / (2 * linearActuatorTopToPivot * linearActuatorBottomToPivot));
    return triangleAngle + angleOffsetRadians;
  }

  private static double linearActuatorFromPitch(double pitchRadians) {
    double angleOffsetRadians = 0.35;
    double linearActuatorExtensionOffset = -168;
    double triangleAngle = pitchRadians + angleOffsetRadians;
    double linearActuatorTopToPivot = 159;
    double linearActuatorBottomToPivot = 251;
    // a = sqrt(b^2 + c^2 - 2bccos(A))
    double linearActuatorLength =
        Math.sqrt(
            linearActuatorTopToPivot * linearActuatorTopToPivot
                + linearActuatorBottomToPivot * linearActuatorBottomToPivot
                - 2
                    * linearActuatorTopToPivot
                    * linearActuatorBottomToPivot
                    * Math.cos(triangleAngle));
    return linearActuatorLength + linearActuatorExtensionOffset;
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
