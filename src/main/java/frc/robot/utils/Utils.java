package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.FieldConstants;

public class Utils {
  public static double scaleAxis(double axis, int power) {
    if (power == 0) return 0;
    if (power < 0) return 0;
    return Math.abs(Math.pow(axis, power)) * Math.signum(axis);
  }

  public static double deadband(double value, double amount) {
    return Math.abs(value) < Math.abs(amount) ? 0 : value;
  }

  public static double clamp(double value, double high, double low) {
    if (high > low) return clamp(value, low, high);
    if (value > high) return high;
    if (value < low) return low;
    return value;
  }

  public static double lutLerp(double[] lut, double xMin, double xStep, double x) {
    int n = (int) Math.floor(((x - xMin) / xStep));
    int nMax = lut.length - 2;
    if (n < 0) {
      n = 0;
    } else if (n >= nMax) {
      n = nMax;
    }
    double xLow = xMin + (n * xStep);
    double t = (x - xLow) / xStep;
    if (t < 0) t = 0;
    if (t > 1) t = 1;
    return lut[n] + t * (lut[n + 1] - lut[n]);
  }

  /** Steps {@code current} toward {@code desired} by at most {@code maxDelta}. */
  public static double clampStep(double current, double desired, double maxDelta) {
    double diff = desired - current;
    if (Math.abs(diff) <= maxDelta) return desired;
    return current + Math.copySign(maxDelta, diff);
  }

  public static Pose2d invertLeftRight(Pose2d pose) {
    return new Pose2d(
        pose.getX(), FieldConstants.fieldWidth - pose.getY(), pose.getRotation().unaryMinus());
  }
}
