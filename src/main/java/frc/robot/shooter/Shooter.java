package frc.robot.shooter;


import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Robot;
import frc.robot.commands.CommandScheduler;
import frc.robot.commands.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    private final XboxController controller;

    public Shooter(XboxController controller, CommandScheduler scheduler) {
        super(scheduler);
        this.controller = controller;
        io = switch (Robot.currentMode) {
            case SIM -> new ShooterIOSim();
            case REAL -> new ShooterIOSpark();
            default -> new ShooterIO() {};
        };
    }

    /**
     * sets the power of the shooter motor
     * @param power Motor power from 0-1
     */
    public void setShooter(double power) {
        io.setFlywheelOpenLoop(power * 12.0);
    }

    /**
     * sets angle of turret
     * @param angle Angle in radians relative to drivetrain rotation (0 = front, pi/2 = right)
     */
    public void setPan(double angle) {
        io.setTurretPitch(angle);
//        final var panMax = (380d / 180d) * Math.PI;
//        final var panMin = (-20d / 180d) * Math.PI;
//        var encoder = motorPan.getAbsoluteEncoder();
//        var currentAngle = (encoder.getPosition() % 1d) * Math.PI * 2;
//        var tolerance = (0.5d / 180d) * Math.PI; // shooter pan angle tolerance in degrees
//        while (Math.abs(currentAngle - angle) <= tolerance) {
//            motorPan.set(Math.signum(angle - currentAngle) * Math.min(Math.abs(angle - currentAngle), 1d));
//            if (currentAngle < panMin || currentAngle > panMax) {
//                throw new IllegalStateException("Pan motor outside of bounds");
//            }
//        }
    }

    /**
     * sets angle of elevation of the turret
     * @param angle angle of elevation in radians. 0 = flat, 90 = straight up
     */
    public void setTilt(double angle) {
        io.setTurretYaw(angle);
        // ASK THE PEOPLE ON THE SHOOTER DESIGN TEAM BEFORE CHANGING THESE NUMBERS
//        final var tiltMax = (35.881d / 180d) * Math.PI;
//        final var tiltMin = (10.17d / 180d) * Math.PI;
//        var encoder = motorTilt.getAbsoluteEncoder();
//        var currentAngle = (encoder.getPosition() % 1d) * Math.PI * 2;
//        var tolerance = (0.5d / 180d) * Math.PI; // shooter tilt angle tolerance in degrees
//        while (Math.abs(currentAngle - angle) <= tolerance) {
//            motorTilt.set(Math.signum(angle - currentAngle) * Math.min(Math.abs(angle - currentAngle), 1d));
//            if (currentAngle < tiltMin || currentAngle > tiltMax) {
//                throw new IllegalStateException("Tilt motor outside of bounds");
//            }
//        }
    }

    public void shootAtTarget(Translation3d target, Pose3d current) {
        var dx = target.getX() - current.getX();
        var dy = target.getY() - current.getY();

        // calculate angle
        // tan(θ) = dy/dx
        double panAngle = Math.atan(dy/dx);

        // TODO: calculate tilt angle and power (misha's job)

        this.setPan(panAngle);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
    }
}

