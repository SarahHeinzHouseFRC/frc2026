package frc.robot.utils;

public class DoubleEncoder {
  private static double closestPair(double[] a, double[] b, double tolerance) {
    int i = 0, j = 0;

    double minDiff = Double.MAX_VALUE;
    double bestA = Double.NaN, bestB = Double.NaN;

    while (i < a.length && j < b.length) {
      double x = a[i];
      double y = b[j];

      double diff = Math.abs(x - y);
      if (diff < minDiff) {
        minDiff = diff;
        bestA = x;
        bestB = y;
      }

      // move pointer on smaller value
      if (x < y) {
        i++;
      } else {
        j++;
      }
    }

    if (!Double.isFinite(bestA) || !Double.isFinite(bestB) || Math.abs(bestA - bestB) > tolerance) {
      return Double.NaN;
    } else {
      return (bestA + bestB) / 2;
    }
  }

  // Same as MathUtil.inputModulus
  private static double inputModulus(double input, double minimumInput, double maximumInput) {
    double modulus = maximumInput - minimumInput;

    // Wrap input if it's above the maximum input
    int numMax = (int) ((input - minimumInput) / modulus);
    input -= numMax * modulus;

    // Wrap input if it's below the minimum input
    int numMin = (int) ((input - maximumInput) / modulus);
    input -= numMin * modulus;

    return input;
  }

  public static double processEncoderValues(double encoder28Radians, double encoder26Radians) {
    double ratio28 = 28d / 200; // -5 to 5 -> +-140t
    double ratio26 = 26d / 200; // -6 to 6 -> +-156t

    encoder28Radians = inputModulus(encoder28Radians, 0, 2 * Math.PI);
    encoder26Radians = inputModulus(encoder26Radians, 0, 2 * Math.PI);

    double[] a = new double[13];
    double[] b = new double[14];

    int iOffset = 6;
    if (encoder28Radians > Math.PI) {
      // furthest negative
      a[0] = (((-7d) * 2 * Math.PI) + encoder28Radians) * ratio28;
      iOffset += 1;
    } else {
      // furthest positive
      a[12] = (((6d) * 2 * Math.PI) + encoder28Radians) * ratio28;
    }
    for (int i = -6; i <= 5; i++) {
      a[i + iOffset] = ((((double) i) * 2 * Math.PI) + encoder28Radians) * ratio28;
    }

    int jOffset = 7;
    for (int j = -7; j <= 6; j++) {
      b[j + jOffset] = ((((double) j) * 2 * Math.PI) + encoder26Radians) * ratio26;
    }

    return closestPair(a, b, .03);
  }
}
