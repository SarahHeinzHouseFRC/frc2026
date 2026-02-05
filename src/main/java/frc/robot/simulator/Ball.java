package frc.robot.simulator;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.FieldConstants;

public class Ball {
  private Vector3d v;
  private Vector3d x;
  private final double mass = BallConstants.ballMass();

  public Ball(Pose3d initialPose, double initialVelocity) {
    double x = initialPose.getX();
    double y = initialPose.getY();
    double z = initialPose.getZ();
    Rotation3d rot = initialPose.getRotation();
    double pitch = rot.getY(); // rad
    double yaw = rot.getZ(); // rad
    double vx = initialVelocity * Math.cos(yaw) * Math.cos(pitch);
    double vy = initialVelocity * Math.sin(yaw) * Math.cos(pitch);
    double vz = initialVelocity * Math.sin(pitch);
    reset(new Vector3d(x, y, z), new Vector3d(vx, vy, vz));
  }

  public Ball(Vector3d initialPosition, Vector3d initialVelocity) {
    reset(initialPosition, initialVelocity);
  }

  public void reset() {
    x = Vector3d.ZERO;
    v = Vector3d.ZERO;
  }

  public void reset(Vector3d initialPosition, Vector3d initialVelocity) {
    x = initialPosition;
    v = initialVelocity;
  }

  protected void simulate(double dt) {
    //        System.out.println(dt);
    Vector3d a = calculateForces().times(1 / mass);
    v = a.times(dt).plus(v);
    x = v.times(dt).plus(x);
  }

  private Vector3d calculateForces() {
    Vector3d F_g = new Vector3d(0, 0, -9.80665 * mass);

    Vector3d F_d = v.times(BallConstants.DRAG * v.magnitude());

    Vector3d F_magnus = new Vector3d(0, 0, 0); // TODO

    return F_g.plus(F_d).plus(F_magnus);
  }

  public Translation3d getPosition() {
    return new Translation3d(x.x(), x.y(), x.z());
  }

  protected boolean shouldDie() {
    if (x.z() < 0) {
      System.out.println("ERROR! BALL TOUCHED GROUND!!!!!!!");
    }
    return (x.z() < 0)
        || (BallConstants.GOAL_POSITION.getDistance(getPosition()) < .3
            && Math.abs(BallConstants.GOAL_POSITION.getZ() - x.z()) < .05);
  }
}

class BallConstants {
  public static final Translation3d GOAL_POSITION =
      FieldConstants.HUB.plus(new Translation3d(0, 0, -.3));
  public static final double DRAG_RHO = 1.225; // (kg/m3)
  public static final double DRAG_A = Math.PI * .075 * .075; // pi * .075^2 (m2)
  public static final double DRAG_C = 0.42; // drag coefficient
  public static final double DRAG = -0.5 * DRAG_RHO * DRAG_A * DRAG_C;
  public static final double BALL_MASS_MIN = 0.203; // (kg)
  public static final double BALL_MASS_MAX = 0.227; // (kg)

  public static double ballMass() {
    return Math.random() * (BALL_MASS_MAX - BALL_MASS_MIN) + BALL_MASS_MIN;
  }
}
