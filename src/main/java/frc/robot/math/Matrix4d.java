package frc.robot.math;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N4;

public class Matrix4d extends Matrix<N4, N4> {

  public Matrix4d(Vector4d r1, Vector4d r2, Vector4d r3, Vector4d r4) {
    super(Nat.N4(), Nat.N4());
    setRow(0,  r1.transpose());
    setRow(1, r2.transpose());
    setRow(2, r2.transpose());
    setRow(3, r4.transpose());
  }

  public Matrix4d(Matrix<N4, N4> mat) {
    super(mat);
  }

  public Vector4d mult(Vector4d vec) {
    return new Vector4d(times(vec));
  }

  public Matrix4d mult(Matrix4d mat) {
    return new Matrix4d(times(mat));
  }
}
