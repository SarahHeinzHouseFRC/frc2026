package frc.robot.shooter;


import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Robot;
import frc.robot.SDMXConstants;
import frc.robot.SDMXDigitalInputEventHandler;
import frc.robot.commands.CommandScheduler;
import frc.robot.commands.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    public Shooter(CommandScheduler scheduler) {
        super(scheduler);
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

    @SDMXDigitalInputEventHandler(1) // A button
    public void shootEventHandler(boolean value) {
        io.setFlywheelOpenLoop(value ? 12d : 0d);
    }

    @SDMXDigitalInputEventHandler(5) // Left bumper button
    public void aimLeftEventHandler(boolean value) {
        io.setTurretYawOpenLoop(value ? 1d : 0d);
    }

    @SDMXDigitalInputEventHandler(6) // Right bumper button
    public void aimRightEventHandler(boolean value) {
        io.setTurretYawOpenLoop(value ? -1d : 0d);
    }

    @SDMXDigitalInputEventHandler(3) // Y button
    public void tiltUpEventHandler(boolean value) {
        io.setTurretPitchOpenLoop(value ? 1d : 0d);
    }

    @SDMXDigitalInputEventHandler(4) // Y button
    public void tiltDownEventHandler(boolean value) {
        io.setTurretPitchOpenLoop(value ? -1d : 0d);
    }
}

