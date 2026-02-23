package frc.robot.utils;

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
}
