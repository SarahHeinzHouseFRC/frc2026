package frc.robot.simulator;


import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class Simulator {
    private long lastTime = -1;
    private SimulatorThread thread;
    private BallSim ballSim = new BallSim();
    private DriveSim driveSim = new DriveSim();

    private static Simulator instance = null;

    public static Simulator getInstance() {
        return instance;
    }

    public static void init() {
        instance = new Simulator();
    }

    private Simulator() {
        thread = new SimulatorThread(this);
        thread.start();
        System.out.println("Simulator started");
    }

    public Pose3d getTruePosition() {
        return new Pose3d(driveSim.getTruePosition());
    }

    public void setSwerveModuleStates(SwerveModuleState[] states) {
        driveSim.setModuleStates(states);
    }

    public Translation3d[] getBallPositions() {
        return ballSim.getBallPositions();
    }

    public void shootBallFromRobot(double pitchAngle, double yawAngle, double velocity) {
        Translation3d position = getTruePosition().getTranslation().plus(new Translation3d(0, 0, .5));
        ballSim.shootBall(new Pose3d(position, new Rotation3d(0, pitchAngle, yawAngle)), velocity);
    }

    public void shootBallFromPosition(Pose3d positionPose, double pitchAngle, double yawAngle, double velocity) {
        Translation3d position = positionPose.getTranslation().plus(new Translation3d(0, 0, .5));
        ballSim.shootBall(new Pose3d(position, new Rotation3d(0, pitchAngle, yawAngle)), velocity);
    }

    private void periodic() {
//        System.out.println("Simulator periodic");
        double dt = 0;
        if (lastTime > 0) {
            dt = (System.nanoTime() - lastTime) / 1_000_000_000.0;
        }
        lastTime = System.nanoTime();
        driveSim.periodic(dt);
        ballSim.periodic(dt);
    }

    protected void run() {
        long LOOP_NS = 1_000_000;
        while (true) {
            long start = System.nanoTime();

            periodic();

            long elapsed = System.nanoTime() - start;

            if (elapsed > LOOP_NS) {
                System.out.printf("WARNING: Simulator took too long to run a cycle: %.3f ms\n", elapsed / 1_000_000.0);
            }

            // busy sleep
            while (System.nanoTime() - start < LOOP_NS);
        }
    }

}

class SimulatorThread extends Thread {
    private final Simulator simulator;
    public SimulatorThread(Simulator simulator) {
        this.simulator = simulator;
    }

    @Override
    public void run() {
        simulator.run();
    }
}
