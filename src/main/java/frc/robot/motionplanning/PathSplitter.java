package frc.robot.motionplanning;

import frc.robot.math.Vector2d;
import java.util.ArrayList;
import org.apache.commons.math3.analysis.interpolation.HermiteInterpolator;

public class PathSplitter {
  public final Polynomial x;
  public final Polynomial y;

  public PathSplitter(Polynomial x, Polynomial y) {
    this.x = x;
    this.y = y;
  }

  public ArrayList<Vector2d> splitPath(double d) {
    ArrayList<Vector2d> path = new ArrayList<Vector2d>();
    double t = 0;
    while (t < 1) {
      Polynomial solution =
          Polynomial.add(
              Polynomial.add(
                  Polynomial.square(Polynomial.add(x, -x.at(t))),
                  Polynomial.negate(Polynomial.square(Polynomial.add(y, -y.at(t))))),
              -d * d);
      path.add(new Vector2d(x.at(t), y.at(t)));
      t = solution.solveMinRoot(t);
    }
    path.add(new Vector2d(x.at(t), y.at(t)));
    return path;
  }

  public ArrayList<Vector2d> splitPathBetter(double segments, ArrayList<Vector2d> points) {
    HermiteInterpolator interpolator = new HermiteInterpolator();
    double t = 0;
    for (Vector2d e : points) {
      interpolator.addSamplePoint(t, new double[] {e.x(), e.y()});
      t += 1;
    }
    ArrayList<Vector2d> path_points = new ArrayList<Vector2d>();
    for (double i = 0; i <= points.size(); i += points.size() / segments) {
      double[] p = interpolator.value(i);
      path_points.add(new Vector2d(p[0], p[1]));
    }
    return points;
  }
}
