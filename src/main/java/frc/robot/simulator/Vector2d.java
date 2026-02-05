package frc.robot.simulator;

public class Vector2d {
  private final double x, y;

  public Vector2d(double x, double y) {
    this.x = x;
    this.y = y;
  }

  public static final Vector2d ZERO = new Vector2d(0, 0);

  public Vector2d plus(Vector2d other) {
    return new Vector2d(x + other.x, y + other.y);
  }

  public Vector2d times(double scalar) {
    return new Vector2d(x * scalar, y * scalar);
  }

  public double x() {
    return x;
  }

  public double y() {
    return y;
  }

  public double magnitude() {
    return Math.sqrt(x * x + y * y);
  }
}
