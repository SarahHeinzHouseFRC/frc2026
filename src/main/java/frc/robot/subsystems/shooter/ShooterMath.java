import org.joml.Vector3d;
import org.joml.Vector2d;
import org.joml.Matrix3d;

public class ShooterMath {
  /**
   * Calculates force of gravity
   * 
   * @param m mass of object
   * @param g gravity constant
   * @return force of gravity
   */
  public static double FGravity(double m, double g) {
    return m*g;
  }

  /**
   * Calculates the force of drag
   * 
   * @param rho fluid density
   * @param v velocity
   * @param Cd coefficent of drag
   * @param A surface area of object contacting fluid
   * @return force of drag
   */
  public static double FDrag(double rho, double v, double Cd, double A) {
    return 0.5*rho*v*v*Cd*A;
  }

  /**
   * Calculates the force of drag assuming the object is a perfect sphere
   * 
   * @param rho fluid density
   * @param v velocity
   * @param radius radius of sphere
   * @return force of drag
   */
  public static double FDragSphere(double rho, double v, double radius) {
    return 0.5*rho*v*v*0.47*(4*Math.PI*radius*radius);
  }


  /**
   * Uses a set turret velocity to make a basic prediction of how much velocity is needed in each direction to score the ball assuming no air resistnace or magnus forces
   * 
   * @param s1 position of robot
   * @param v1 velocity of robot
   * @param s2 position of target
   * @param v turret velocity
   * @return velocity in each direction neede to be produced by the shooter to score
   */
  public static Vector3d calcuateForcesModelOne(Vector3d s1, Vector3d v1, Vector3d s2, double v) {
    final double ta = -0.0098/2;
    final double tb = (v+v1.x()+v1.y()+v1.z());
    final double tc = (s2.x()-s1.x()+s2.y()-s1.y()+s2.z()-s1.z());
    final double t = (-tb+Math.sqrt(tb*tb-4*ta*tc))/(2*ta);
    final double a = (s2.x()-s1.x())/t+v1.x();
    final double b = (s2.y()-s1.y())/t+v1.y();
    final double c = v+v1.x()+v1.y()-(s2.x()-s1.x()+s2.y()-s1.y())/t;
    return new Vector3d(a, b, c);
  }

  /**
   * Uses the angle the ball is to enter the goal to calculate the velocity in each direction
   * 
   * @param s1 position of robot
   * @param v1 velocity of robot
   * @param s2 position of target
   * @param alpha angle of goal entry
   * @return velocity in each direction neede to be produced by the shooter to score
   */
  public static Vector3d calcuateForcesModelTwo(Vector3d s1, Vector3d v1, Vector3d s2, double alpha) {
    final double tan_a = Math.tan(alpha);
    final double ta = 0.0098/2;
    final double tb = tan_a;
    final double tc = (s2.x()-s1.x()+s2.y()-s1.y()+s2.z()-s1.z());
    final double t = (-tb+Math.sqrt(tb*tb-4*ta*tc))/(2*ta);
    final double a = (s2.x()-s1.x())/t+v1.x();
    final double b = (s2.y()-s1.y())/t+v1.y();
    final double c = 0.0098*t+tan_a-v1.z()-(s2.x()-s1.x()+s2.y()-s1.y())/t;
    return new Vector3d(a, b, c);
  }

  /**
   * TODO: implement
   * 
   * @param expectedVelocities vector holding expected velocities in all three directions (x, y, z)
   * @param robotRotation 3x3 roattion matirx that converts world coordiantes to robot coordinates
   * @return turret roatation in radians (0 points towards the front of the robot) and angle of elevation in radians
   */
  public static Vector2d calculateTurretAngles(Vector3d expectedVelocities, Matrix3d robotRotation) {

  }
}