package frc.robot.motionplanning;

import frc.robot.math.Vector2d;
import java.util.ArrayList;

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
        while (t<1) {
            Polynomial solution = Polynomial.add(Polynomial.add(Polynomial.square(Polynomial.add(x, -x.at(t))), Polynomial.negate(Polynomial.square(Polynomial.add(y,-y.at(t))))), -d*d);
            path.add(new Vector2d(x.at(t), y.at(t)));
            t = solution.solveMinRoot(t);
        }
        path.add(new Vector2d(x.at(t), y.at(t)));
        return path;
    }
}