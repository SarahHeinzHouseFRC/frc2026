package frc.robot.math;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class Vector3d extends Vector<N3> {
  public Vector3d(double x, double y, double z) {
    super(Nat.N3());
    this.set(0, 0, x);
    this.set(1, 0, y);
    this.set(2, 0, z);
  }

  public Vector3d(Matrix<N3, N1> vec) {
    super(Nat.N3());
    this.set(0, 0, vec.get(0, 0));
    this.set(1, 0, vec.get(1, 0));
    this.set(2, 0, vec.get(2, 0));
  }

  public double x() {
    return this.get(0);
  }

  public double y() {
    return this.get(1);
  }

  public double z() {
    return this.get(2);
  }

  public Matrix<N3, N1> toMatrix() {
    return this.copy();
  }
}