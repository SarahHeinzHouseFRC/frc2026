package frc.robot.shooter;

import static frc.robot.shooter.ShooterConstants.*;

import com.revrobotics.AbsoluteEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.Robot;
import frc.robot.drive.Drive;
import frc.robot.utils.TimestampedDoubleBuffer;
import frc.robot.vision.Vision;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private double shooterSpeedSetpoint = 6000;
  private double autoAimDirection = 0.05;
  private final ShotCalculator shotCalculator = ShotCalculators.iterativeShotCalculator;

  public static Shooter instance;
  private final XboxController controller;

  private final TimestampedDoubleBuffer yawBuffer = new TimestampedDoubleBuffer(10);

  private boolean shooterIsInit = false;

  public static void init(XboxController controller) {
    if (instance != null) {
      throw new IllegalStateException("Shooter instance already initialized.");
    }
    instance = new Shooter(controller);
  }

  public static Shooter getInstance() {
    if (instance == null) {
      throw new IllegalStateException("Shooter instance not initialized.");
    }
    return instance;
  }

  private Shooter(XboxController controller) {
    instance = this;
    io =
        switch (Robot.currentMode) {
          case SIM -> new ShooterIOSim();
          case REAL -> new ShooterIOSpark();
          default -> new ShooterIO() {};
        };

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

  /** Sets shooter wheel target speed in RPM. */
  public void setFlywheelVelocityRpm(double rpm) {
    io.setFlywheelVelocity(rpm);
  }

  /** Returns measured shooter speed in RPM. */
  public double getFlywheelVelocityRpm() {
    return inputs.flywheelVelocityRotationsPerMinute;
  }

  /** True when measured RPM is within tolerance of target RPM. */
  public boolean isFlywheelAtSpeed(double targetRpm, double toleranceRpm) {
    return Math.abs(getFlywheelVelocityRpm() - targetRpm) <= toleranceRpm;
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

  /** Points the turret toward the hub using odometry pose. */
  public void pointAtHub() {
    Pose2d myPose = Drive.getInstance().getPose();
    Rotation2d angle =
        FieldConstants.HUB
            .toTranslation2d()
            .minus(myPose.getTranslation())
            .getAngle()
            .minus(myPose.getRotation())
            .plus(Rotation2d.kPi);
    io.setTurretYaw(MathUtil.angleModulus(angle.getRadians()));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
    yawBuffer.add(inputs.turretYawRadians, inputs.timestamp);
    if (controller.getAButtonPressed()) {
      CommandScheduler.getInstance().schedule(autoAimCommand());
    } else if (controller.getXButtonPressed()) {
      CommandScheduler.getInstance().schedule(directDriveCommand());
    }

    if (!inputs.isTurretInit || DriverStation.isDisabled()) {
      io.recalibrateYaw();
    }
  }

  public boolean isTurretInit() {
    return inputs.isTurretInit;
  }

  public double getYawAtTime(double time) {
    return yawBuffer.getValueAtTimestamp(time);
  }

  public Command directDriveCommand() {
    return Commands.run(
        () -> {
          if (controller.getLeftBumperButton() && controller.getRightBumperButton()) {
            io.recalibrateYaw();
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
                  .minus(myPose.getRotation())
                  .plus(Rotation2d.kPi);

          SmartDashboard.putNumber("Target Yaw Pos", MathUtil.angleModulus(angle.getRadians()));
        },
        this);
  }

  public Command autoAimCommand() {
    return Commands.run(
        () -> {
          if (controller.getLeftBumperButton() && controller.getRightBumperButton()) {
            io.recalibrateYaw();
          }
          Pose2d myPose = Drive.getInstance().getPose();
          Translation2d itsPose = FieldConstants.HUB.toTranslation2d();
          ChassisSpeeds speeds = Drive.getInstance().getChassisSpeeds();
          double angleToHub =
              itsPose
                  .minus(myPose.getTranslation())
                  .getAngle()
                  .minus(myPose.getRotation())
                  .getRadians();
          double vrad =
              -speeds.vxMetersPerSecond * Math.cos(angleToHub)
                  - speeds.vyMetersPerSecond * Math.sin(angleToHub);
          double vtan =
              speeds.vxMetersPerSecond * Math.sin(angleToHub)
                  - speeds.vyMetersPerSecond * Math.cos(angleToHub);
          SmartDashboard.putNumber("vrad", vrad);
          SmartDashboard.putNumber("vtan", vtan);
          double poseOffset =
              switch (Robot.VERSION) {
                case V1 -> .3;
                case V2 -> 0;
              };
          ShotParams shotParams =
              shotCalculator.calculateShotParams(
                  itsPose.getDistance(myPose.getTranslation()) + poseOffset, vrad, vtan);
          Logger.recordOutput(
              "/Shooter/shotParams/yawOffsetRadians", shotParams.yawOffsetRadians());
          Logger.recordOutput(
              "/Shooter/shotParams/flywheelVelocityRotationsPerMinute",
              shotParams.flywheelVelocityRotationsPerMinute());
          Logger.recordOutput(
              "/Shooter/shotParams/linearActuatorExtensionMillimeters",
              shotParams.linearActuatorExtensionMillimeters());
          //                  itsPose.getDistance(myPose.getTranslation()) + .3, vrad, vtan);
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
                  .plus(Rotation2d.kPi)
                  .plus(Rotation2d.fromRadians(shotParams.yawOffsetRadians()));
          SmartDashboard.putNumber("Target Yaw Pos", MathUtil.angleModulus(angle.getRadians()));

          if (controller.getLeftBumperButton() && controller.getRightBumperButton()) {
            io.recalibrateYaw();
          }
          if (!Vision.getInstance().isVisionInit()) {
            double setpoint = getTurretYaw();
            setpoint += autoAimDirection;
            if (setpoint * Math.signum(autoAimDirection) > 1.57) {
              autoAimDirection = -autoAimDirection;
            }
            io.setTurretYaw(setpoint);
          } else {
            double setpoint = MathUtil.inputModulus(angle.getRadians(), yawModuloMax, yawModuloMin);
            io.setTurretYaw(setpoint);
          }
        },
        this);
  }

  public void set28TurretAngleSupplier(AbsoluteEncoder enc) {
    io.set28TurretAngleSupplier(enc);
  }
}
