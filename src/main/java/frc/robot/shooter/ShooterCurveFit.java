package frc.robot.shooter;

public class ShooterCurveFit {

  // Linear x-y
  public static double calculateY(double x) {
    return 1.2339285275318290 * x + 5.5529796030436920;
  }

  // Quintic x-z
  public static double calculateZ(double x) {
    return -0.0000377394240959 * x * x * x * x * x
        + 0.0009654412092112 * x * x * x * x
        + -0.0100328138582404 * x * x * x
        + 0.0544524981486492 * x * x
        + -0.1941103100489582 * x
        + 1.5570202655380678;
  }
}
