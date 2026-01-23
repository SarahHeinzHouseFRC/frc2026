package frc.robot.simulator;

public class DriveSimModule {
    public final DriveSimModuleType type;
    private double speed;
    private double angleRadians;
    public DriveSimModule(DriveSimModuleType type) {
        this.speed = 0.0;
        this.angleRadians = 0.0;
        this.type = type;
    }

    public void setSpeed(double speed) {
        this.speed = speed;
    }

    public void setAngle(double angleRadians) {
        this.angleRadians = angleRadians;
    }
    public Vector2d simulate() {
        // Convert polar coordinates (speed, angle) to Cartesian (x, y)
        double idealForceX = speed * Math.cos(angleRadians);
        double idealForceY = speed * Math.sin(angleRadians);

        // Calculate maximum friction force available
        double mu_k = 1.067;
        double mu_s = 1.067;
        double maxFriction = mu_s * 511.19838; // 115 lb to kg * 9.8

        // Clamp the force to available traction
        double idealForceMagnitude = Math.sqrt(idealForceX * idealForceX + idealForceY * idealForceY);

        if (idealForceMagnitude > maxFriction) {
            // Wheel is slipping - scale down to max available friction
            double scale = maxFriction / idealForceMagnitude;
            idealForceX *= scale;
            idealForceY *= scale;
        }


        return new Vector2d(idealForceX, idealForceY);
    }
}

