package frc.robot.math;

public class Transformation {
  Matrix4d mat;

  public Transformation(Matrix4d mat) {
    this.mat = mat;
  }
   
  public Matrix3d getRotation() {
    return new Matrix3d(new Vector4d(mat.extractRowVector(0).transpose()).get3d(), 
                          new Vector4d(mat.extractRowVector(1).transpose()).get3d(), 
                          new Vector4d(mat.extractRowVector(2).transpose()).get3d());
  }

  public Vector3d getTranslation() {
    return new Vector3d(mat.get(0, 3), mat.get(1, 3), mat.get(2, 3));
  }
}