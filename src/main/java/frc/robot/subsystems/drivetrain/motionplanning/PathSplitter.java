package frc.robot.subsystems.drivetrain.motionplanning;

import org.joml.Vector2d;

import java.lang.reflect.Method;

public class PathSplitter {
    public final Polynomial x;
    public final Polynomial y;

    public PathSplitter(Polynomial x, Polynomial y) {
        this.x = x;
        this.y = y;
    }

    public Vector2d[] splitPath(int segments) {
        var points = new Vector2d[segments];
        // TODO
        return points;
    }
}
