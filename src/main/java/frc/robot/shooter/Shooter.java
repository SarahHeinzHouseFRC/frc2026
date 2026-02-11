package frc.robot.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.FieldConstants;
import frc.robot.Robot;
import frc.robot.TimestampedDoubleBuffer;
import frc.robot.commands.Command;
import frc.robot.commands.CommandScheduler;
import frc.robot.commands.Commands;
import frc.robot.commands.SubsystemBase;
import frc.robot.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private double shooterSpeedSetpoint = 6000;

  private final ShotCalculator shotCalculator = ShotCalculators.lutShotCalculator;

  public static Shooter instance;
  private XboxController controller;

  private final TimestampedDoubleBuffer yawBuffer = new TimestampedDoubleBuffer(100);

  public static void init(XboxController controller, CommandScheduler scheduler) {
    if (instance != null) {
      throw new IllegalStateException("Shooter instance already initialized.");
    }
    instance = new Shooter(controller, scheduler);
  }

  public static Shooter getInstance() {
    if (instance == null) {
      throw new IllegalStateException("Shooter instance not initialized.");
    }
    return instance;
  }

  private Shooter(XboxController controller, CommandScheduler scheduler) {
    super(scheduler);
    instance = this;
    io =
        switch (Robot.currentMode) {
          case SIM -> new ShooterIOSim();
          case REAL -> new ShooterIOSpark();
          default -> new ShooterIO() {};
        };
//    setDefaultCommand(directDriveCommand());
        setDefaultCommand(autoAimCommand());
    this.controller = controller;
  }

  /**
   * sets the power of the shooter motor
   *
   * @param power Motor power from 0-1
   */
  public void setShooter(double power) {
    io.setFlywheelOpenLoop(power * 12.0);
  }

  /**
   * sets angle of turret
   *
   * @param angle Angle in radians relative to drivetrain rotation (0 = front, pi/2 = right)
   */
  public void setPan(double angle) {
    //    io.setTurretPitch(angle);
    //        final var panMax = (380d / 180d) * Math.PI;
    //        final var panMin = (-20d / 180d) * Math.PI;
    //        var encoder = motorPan.getAbsoluteEncoder();
    //        var currentAngle = (encoder.getPosition() % 1d) * Math.PI * 2;
    //        var tolerance = (0.5d / 180d) * Math.PI; // shooter pan angle tolerance in degrees
    //        while (Math.abs(currentAngle - angle) <= tolerance) {
    //            motorPan.set(Math.signum(angle - currentAngle) * Math.min(Math.abs(angle -
    // currentAngle), 1d));
    //            if (currentAngle < panMin || currentAngle > panMax) {
    //                throw new IllegalStateException("Pan motor outside of bounds");
    //            }
    //        }
  }

  /**
   * sets angle of elevation of the turret
   *
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
    //            motorTilt.set(Math.signum(angle - currentAngle) * Math.min(Math.abs(angle -
    // currentAngle), 1d));
    //            if (currentAngle < tiltMin || currentAngle > tiltMax) {
    //                throw new IllegalStateException("Tilt motor outside of bounds");
    //            }
    //        }
  }

  // ccw positive
  public double getTurretYaw() {
    return inputs.turretYawRadians;
  }

  public void shootAtTarget(Translation3d target, Pose3d current) {
    var dx = target.getX() - current.getX();
    var dy = target.getY() - current.getY();

    // calculate angle
    // tan(theta) = dy/dx
    double panAngle = Math.atan(dy / dx);

    // TODO: calculate tilt angle and power (misha's job)

    this.setPan(panAngle);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
    SmartDashboard.putNumber(
        "shooter speed rpm", ((double) 13 / 9) * inputs.flywheelVelocityRotationsPerMinute);
    yawBuffer.add(inputs.turretYawRadians, inputs.timestamp);
  }

  public double getYawAtTime(double time) {
    return yawBuffer.getValueAtTimestamp(time);
  }

  private Command directDriveCommand() {
    return Commands.run(
        () -> {
          if (controller.getLeftBumperButton() && controller.getRightBumperButton()) {
            io.zeroYaw();
          }
          int pov = controller.getPOV();
          boolean right = pov == 45 || pov == 90 || pov == 135;
          boolean down = pov == 135 || pov == 180 || pov == 225;
          boolean left = pov == 225 || pov == 270 || pov == 315;
          boolean up = pov == 315 || pov == 360 || pov == 0 || pov == 45;

          if (up) {
            io.setTurretPitchOpenLoop(12);
          }
          if (down) {
            io.setTurretPitchOpenLoop(-12);
          }
          if (left) {
            io.setTurretYawOpenLoop(1d);
          }
          if (right) {
            io.setTurretYawOpenLoop(-1d);
          }

          if (!left && !right) {
            io.setTurretYawOpenLoop(0d);
          }
          shooterSpeedSetpoint =
              MathUtil.clamp(
                  shooterSpeedSetpoint - 50 * MathUtil.applyDeadband(controller.getRightY(), .1),
                  0,
                  6000);
          SmartDashboard.putNumber("shooterSpeedSetpoint", shooterSpeedSetpoint);
          if (controller.getRightBumperButton()) {
            io.setFlywheelVelocity(shooterSpeedSetpoint);
          } else {
            io.setFlywheelOpenLoop(0);
          }
          Pose2d myPose = Drive.getInstance().getPose();
          Translation2d itsPose = FieldConstants.HUB.toTranslation2d();
          Rotation2d angle =
              itsPose
                  .minus(myPose.getTranslation())
                  .getAngle()
                  .minus(myPose.getRotation()).plus(Rotation2d.kPi);

          SmartDashboard.putNumber("Target Yaw Pos", MathUtil.angleModulus(angle.getRadians()));
        },
        this);
  }

  private Command autoAimCommand() {
    return Commands.run(
        () -> {
          if (controller.getLeftBumperButton() && controller.getRightBumperButton()) {
            io.zeroYaw();
          }
          Pose2d myPose = Drive.getInstance().getPose();
          Translation2d itsPose = FieldConstants.HUB.toTranslation2d();
          ShotParams shotParams =
              shotCalculator.calculateShotParams(
                  itsPose.getDistance(myPose.getTranslation()) + .3, 0, 0);
          io.setLinearActuatorPosition(shotParams.linearActuatorExtensionMillimeters());
          if (controller.getRightBumperButton() || controller.getRightTriggerAxis() > .1) {
            io.setFlywheelVelocity(shotParams.flywheelVelocityRotationsPerMinute());
          } else {
            io.setFlywheelOpenLoop(0);
          }
          Rotation2d angle =
              itsPose
                  .minus(myPose.getTranslation())
                  .getAngle()
                  .minus(myPose.getRotation())
                  .plus(Rotation2d.kPi);
          SmartDashboard.putNumber("Target Yaw Pos", MathUtil.angleModulus(angle.getRadians()));
          io.setTurretYaw(MathUtil.angleModulus(angle.getRadians()));
          if (controller.getLeftBumperButton() && controller.getRightBumperButton()) {
            io.zeroYaw();
          }
        },
        this);
  }

  //    @SDMXAnalogInputEventHandler(2) // Left trigger
  //    public static void shootEventHandler(byte value) {
  //        Shooter.instance.io.setFlywheelOpenLoop(((int)value/256d)*12d);
  //    }
  //
  //    @SDMXDigitalInputEventHandler(5) // Left bumper button
  //    public static void aimLeftEventHandler(boolean value) {
  //        Shooter.instance.io.setTurretYawOpenLoop(value ? 1d : 0d);
  //    }
  //
  //    @SDMXDigitalInputEventHandler(6) // Right bumper button
  //    public static void aimRightEventHandler(boolean value) {
  //        Shooter.instance.io.setTurretYawOpenLoop(value ? -1d : 0d);
  //    }
  //
  //    @SDMXDigitalInputEventHandler(4) // Y button
  //    public static void tiltUpEventHandler(boolean value) {
  //        // Shooter.instance.io.setTurretPitchOpenLoop(value ? 12d : 0d);
  //        Shooter.instance.io.setTurretPitch((1d/4));
  //    }
  //
  //    @SDMXDigitalInputEventHandler(3) // X button
  //    public static void tiltDownEventHandler(boolean value) {
  //        // Shooter.instance.io.setTurretPitchOpenLoop(value ? -6d : 0d);
  //        Shooter.instance.io.setTurretPitch(0);
  //    }
}
