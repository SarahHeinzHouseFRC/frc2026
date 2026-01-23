package frc.robot.math;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N3;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.opencv.core.Mat;

public class Matrix3d extends Matrix<N3, N3> {
  public Matrix3d(Vector3d r1, Vector3d r2, Vector3d r3) {
    super(Nat.N3(), Nat.N3());
    setRow(0,  r1.transpose());
    setRow(1, r2.transpose());
    setRow(2, r2.transpose());
  }

  public static Matrix3d fromYaw(double a) {
    return new Matrix3d(
      new Vector3d(Math.cos(a), -Math.sin(a), 0), 
      new Vector3d(Math.sin(a), -Math.cos(a), 0), 
      new Vector3d(0,0, 1)
    );
  }

  public static Matrix3d fromPitch(double a) {
    return new Matrix3d(
      new Vector3d(1, 0, 0), 
      new Vector3d(0, Math.cos(a), -Math.sin(a)), 
      new Vector3d(0, Math.sin(a), Math.cos(a))
    );
  }

  public Matrix3d(Matrix<N3, N3> mat) {
    super(mat);
  }

  public Vector3d mult(Vector3d vec) {
    return new Vector3d(times(vec));
  }

  public Matrix3d mult(Matrix3d mat) {
    return new Matrix3d(times(mat));
  }
}