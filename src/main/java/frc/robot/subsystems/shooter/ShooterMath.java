

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

  public static void CalcuateForces() {
    
  }
}