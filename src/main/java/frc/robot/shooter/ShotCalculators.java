package frc.robot.shooter;

import frc.robot.Robot;
import frc.robot.utils.Utils;

public class ShotCalculators {

  private ShotCalculators() {}

  private static final double[] rpmLut =
      switch (Robot.VERSION) {
        case V1 -> new double[] {2950, 2950, 2950, 3200, 3300, 3500, 3600, 3800, 3950, 4150};
        case V2 -> new double[] {2800, 3300, 3750, 3950, 3950, 4000, 4000, 4300, 4300};
      };
  private static final double[] linearLut =
      switch (Robot.VERSION) {
        case V1 -> new double[] {15, 15, 15, 25, 30, 35, 45, 45, 55, 60};
        case V2 -> new double[] {0, 0, 0, 0, 10, 15, 20, 20, 22.5};
      };
  private static final double[] timeLut =
      switch (Robot.VERSION) {
        case V1 -> null;
        case V2 -> new double[] {0.83, 1.12, 1.32, 1.39, 1.37, 1.34, 1.37, 1.49, 1.48};
      };

  private static final double timeDelay =
      switch (Robot.VERSION) {
        case V1 -> 0.0;
        case V2 -> 0.1;
      };

  private static final double lutStart = 1.5;
  private static final double lutStep = .5;

  public static final ShotCalculator iterativeShotCalculator =
      (distanceMeters, velocityRadialMetersPerSecond, velocityTangentialMetersPerSecond) -> {
        double distanceRadial = distanceMeters;
        double distanceTangential = 0;
        double rpm = 0;
        double linear = 15;
        double iMax = 64;
        double lastTime = 0;
        for (int i = 0; i < iMax; i++) {
          float distance = (float) Math.hypot(distanceRadial, distanceTangential);

          rpm = Utils.lutLerp(rpmLut, lutStart, lutStep, distance);
          linear = Utils.lutLerp(linearLut, lutStart, lutStep, distance);
          if (timeLut == null) {
            System.out.println("[WARNING] timeLut is null");
            break;
          }
          double time = Utils.lutLerp(timeLut, lutStart, lutStep, distance) + timeDelay;
          if (Math.abs(lastTime - time) > .01 && i == iMax - 1) {
            System.out.println("[WARNING] did not converge iter");
          }
          lastTime = time;

          // move robot and calc new params
          distanceRadial = distanceMeters + velocityRadialMetersPerSecond * time;
          distanceTangential = velocityTangentialMetersPerSecond * time;
        }

        return new ShotParams(rpm, linear, Math.atan2(distanceTangential, distanceRadial));
      };

  public static final ShotCalculator lutShotCalculator =
      (distanceMeters, velocityRadialMetersPerSecond, velocityTangentialMetersPerSecond) -> {
        double lerpRpm = Utils.lutLerp(rpmLut, lutStart, lutStep, distanceMeters);
        double lerpLinear = Utils.lutLerp(linearLut, lutStart, lutStep, distanceMeters);

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

  public static double pitchFromLinearActuator(double linearActuatorExtensionMillimeters) {
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

  public static double linearActuatorFromPitch(double pitchRadians) {
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

  public static double velocityFromFlywheel(double flywheelVelocityRotationsPerMinute) {
    return flywheelVelocityRotationsPerMinute
        / 60 /* -> rotations per second */
        * (2 * Math.PI) /* -> radians per second */
        * 0.0508 /* radius in meters -> meters per second */
        * flywheelEfficiencyRatio;
  }

  public static double flywheelFromVelocity(double velocityMetersPerSecond) {
    return velocityMetersPerSecond
        / flywheelEfficiencyRatio
        / 0.0508 /* radius in meters -> radians per second */
        / (2 * Math.PI) /* radians per second -> rotations per second */
        * 60 /* -> rotations per minute */;
  }
}
