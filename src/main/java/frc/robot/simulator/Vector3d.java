package frc.robot.simulator;

public class Vector3d {
    private final double x, y, z;
    public Vector3d(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }
    public static final Vector3d ZERO = new Vector3d(0, 0, 0);
    public Vector3d plus(Vector3d other) {
        return new Vector3d(x + other.x, y + other.y, z + other.z);
    }
    public Vector3d times(double scalar) {
        return new Vector3d(x * scalar, y * scalar, z * scalar);
    }
    public double x() { return x; }
    public double y() { return y; }
    public double z() { return z; }
    public double magnitude() { return Math.sqrt(x*x + y*y + z*z); }
}
