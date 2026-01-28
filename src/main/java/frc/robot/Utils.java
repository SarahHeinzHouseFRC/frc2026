package frc.robot;


public class Utils {
    public static double scaleAxis(double axis, int power) {
        if (power == 0) return 0;
        if (power < 0) return 0;
        return Math.abs(Math.pow(axis, power)) * Math.signum(axis);
    }

    public static double deadband(double value, double amount) {
        return Math.abs(value) < Math.abs(amount) ? 0 : value;
    }
}
