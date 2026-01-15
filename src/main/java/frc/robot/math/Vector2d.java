package frc.robot.math;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;

public class Vector2d extends Vector<N3> {
  public Vector2d(double x, double y) {
    super(Nat.N3());
    this.set(0, 0, x);
    this.set(1, 0, y);
  }

  public double x() {
    return this.get(0);
  }

  public double y() {
    return this.get(1);
  }
}