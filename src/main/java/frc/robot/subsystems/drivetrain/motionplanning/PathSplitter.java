package frc.robot.subsystems.drivetrain.motionplanning;

import org.joml.Vector2d;

import java.lang.reflect.Method;
import java.util.ArrayList;

public class PathSplitter {
    public final Polynomial x;
    public final Polynomial y;

    public PathSplitter(Polynomial x, Polynomial y) {
        this.x = x;
        this.y = y;
    }

    public ArrayList<Vector2d> splitPath(double distance) {
        double t=0;
        ArrayList<Vector2d> path = new ArrayList<Vector2d>();
        path.add(new Vector2d(x.at(t), y.at(t)));
        Polynomial solver = (Polynomial.add())
        while (true) {

        }
        return path;
    }
}
