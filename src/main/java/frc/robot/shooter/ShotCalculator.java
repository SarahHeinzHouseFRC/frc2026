package frc.robot.shooter;

@FunctionalInterface
public interface ShotCalculator {
  ShotParams calculateShotParams(
      double distanceMeters,
      double velocityRadialMetersPerSecond,
      double velocityTangentialMetersPerSecond);
}
