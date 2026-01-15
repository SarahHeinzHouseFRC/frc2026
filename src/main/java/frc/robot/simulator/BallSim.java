package frc.robot.simulator;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;

import java.util.ArrayList;
import java.util.Iterator;

public class BallSim {
    private final ArrayList<Ball> balls = new ArrayList<>();
    private final Object ballLock = new Object();
    protected BallSim() {

    }

    protected void periodic(double dt) {
        synchronized (ballLock) {
            Iterator<Ball> it = balls.iterator();
            while (it.hasNext()) {
                Ball ball = it.next();
                if (ball.shouldDie()) {
                    it.remove(); // safe
                } else {
                    ball.simulate(dt);
                }
            }
        }
    }

    public void shootBall(Pose3d initialPose, double initialVelocity) {
        shootBall(new Ball(initialPose, initialVelocity));
    }

    public void shootBall(double x, double y, double z, double vx, double vy, double vz) {
        shootBall(new Ball(new Vector3d(x, y, z), new Vector3d(vx, vy, vz)));
    }

    private void shootBall(Ball ball) {
        synchronized (ballLock) {
            balls.add(ball);
        }
    }

    public Translation3d[] getBallPositions() {
        synchronized (ballLock) {
            int size = balls.size();
            Translation3d[] translation3ds = new Translation3d[size];
            for (int i = 0; i < size; i++) {
                translation3ds[i] = balls.get(i).getPosition();
            }
            return translation3ds;
        }
    }
}
