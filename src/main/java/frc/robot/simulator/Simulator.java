package frc.robot.simulator;



public class Simulator {
    private long lastTime = -1;
    private SimulatorThread thread;
    private BallSim ballSim = new BallSim();

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

    private void periodic() {
//        System.out.println("Simulator periodic");
        double dt = 0;
        if (lastTime > 0) {
            dt = (System.nanoTime() - lastTime) / 1_000_000_000.0;
        }
        lastTime = System.nanoTime();
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

    public BallSim getBallSim() {
        return ballSim;
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
