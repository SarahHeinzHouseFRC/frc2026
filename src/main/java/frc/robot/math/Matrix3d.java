package frc.robot.math;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N3;

public class Matrix3d extends Matrix<N3, N3> {
  public Matrix3d(Vector3d r1, Vector3d r2, Vector3d r3) {
    super(Nat.N3(), Nat.N3());
    this.setRow(0,  r1.transpose());
    this.setRow(1, r2.transpose());
    this.setRow(2, r2.transpose());
  }

  public Vector3d mult(Vector3d vec) {
    return new Vector3d(this.times(vec));
  }
}