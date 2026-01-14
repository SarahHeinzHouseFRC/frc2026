package frc.robot.subsystems.shooter;


import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private final SparkMax motorPan;
    private final SparkMax motorTilt;
    private final SparkMax motorShoot;

    // With eager singleton initialization, any static variables/fields used in the 
    // constructor must appear before the "INSTANCE" variable so that they are initialized 
    // before the constructor is called when the "INSTANCE" variable initializes.

    /**
     * The Singleton instance of this ShooterSubsystem. Code should use
     * the {@link #getInstance()} method to get the single instance (rather
     * than trying to construct an instance of this class.)
     */
    private final static ShooterSubsystem INSTANCE = new ShooterSubsystem();

    /**
     * Returns the Singleton instance of this ShooterSubsystem. This static method
     * should be used, rather than the constructor, to get the single instance
     * of this class. For example: {@code ShooterSubsystem.getInstance();}
     */
    @SuppressWarnings("WeakerAccess")
    public static ShooterSubsystem getInstance() {
        return INSTANCE;
    }

    /**
     * Creates a new instance of this ShooterSubsystem. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    private ShooterSubsystem() {
        // SET DEVICE IDs HERE
        motorPan = new SparkMax(0, SparkLowLevel.MotorType.kBrushless);
        motorTilt = new SparkMax(0, SparkLowLevel.MotorType.kBrushless);
        motorShoot = new SparkMax(0, SparkLowLevel.MotorType.kBrushless);
    }

    /**
     * sets the power of the shooter motor
     * @param power Motor power from 0-1
     */
    public void setShooter(double power) {
        motorShoot.set(power);
    }

    /**
     * sets angle of turret
     * @param angle Angle in degrees relative to drivetrain rotation (0 deg = front, 90 deg = right)
     */
    public void setPan(double angle) {
        var encoder = motorPan.getAbsoluteEncoder();
        var currentAngle = encoder.getPosition() % 1d;
        var tolerance = 0.5d; // shooter pan angle tolerance in degrees
        while (Math.abs(currentAngle - angle) <= tolerance) {
            motorPan.set(Math.signum(angle - currentAngle) * Math.min(Math.abs(angle - currentAngle), 1d));
        }
    }

    /**
     * sets angle of elevation of the turret
     * @param angle angle of elevation in degrees. 0 deg = flat, 90 deg = straight up
     */
    public void setTilt(double angle) {
        // ASK THE PEOPLE ON THE SHOOTER DESIGN TEAM BEFORE CHANGING THESE NUMBERS
        final var tiltMax = 35.881d;
        final var tiltMin = 10.17d;
        var encoder = motorTilt.getAbsoluteEncoder();
        var currentAngle = encoder.getPosition() % 1d;
        var tolerance = 0.5d; // shooter pan angle tolerance in degrees
        while (Math.abs(currentAngle - angle) <= tolerance) {
            motorTilt.set(Math.signum(angle - currentAngle) * Math.min(Math.abs(angle - currentAngle), 1d));
            if (currentAngle < tiltMin || currentAngle > tiltMax) {
                throw new IllegalStateException("Tilt motor outside of bounds");
            }
        }
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
}

