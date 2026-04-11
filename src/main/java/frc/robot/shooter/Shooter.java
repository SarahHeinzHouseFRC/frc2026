package frc.robot.shooter;

import static frc.robot.shooter.ShooterConstants.*;

import com.revrobotics.AbsoluteEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.Robot;
import frc.robot.drive.Drive;
import frc.robot.utils.DoubleEncoder;
import frc.robot.utils.TimestampedDoubleBuffer;
import frc.robot.vision.Vision;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private double autoAimDirection = 0.05;
  private final ShotCalculator shotCalculator = ShotCalculators.iterativeShotCalculator;

  public static Shooter instance;
  private final XboxController controller;

  private boolean wasZeroForced = false;

  private final TimestampedDoubleBuffer yawBuffer = new TimestampedDoubleBuffer(10);

  @AutoLogOutput private boolean isTurretInit = false;

  @AutoLogOutput public boolean limpMode = false;

  private double lastNonLimpTime = 0.0;

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

  //  /**
  //   * sets the power of the shooter motor
  //   *
  //   * @param power Motor power from 0-1
  //   */
  //  public void setShooter(double power) {
  //    io.setFlywheelOpenLoop(power * 12.0);
  //  }

  /** Sets shooter wheel target speed in RPM. */
  public void setFlywheelVelocityRpm(double rpm) {
    flywheelTarget = rpm;
    io.setFlywheelVelocity(rpm);
  }

  /** Sets flywheel open loop */
  public void setFlywheelOpenLoop(double voltage) {
    flywheelTarget = -1;
    io.setFlywheelOpenLoop(voltage);
  }

  /** Returns measured shooter speed in RPM. */
  public double getFlywheelVelocityRpm() {
    return inputs.flywheelVelocityRotationsPerMinute;
  }

  /** True when measured RPM is within tolerance of target RPM. */
  public boolean isFlywheelAtSpeed(double targetRpm, double toleranceRpm) {
    return Math.abs(getFlywheelVelocityRpm() - targetRpm) <= toleranceRpm;
  }

  private double flywheelTarget = -1;

  public boolean isFlywheelAtSpeed() {
    if (flywheelTarget <= 0) {
      return true;
    }
    return isFlywheelAtSpeed(flywheelTarget, Math.max(flywheelTarget * .1, 300));
  }

  public void stopFlywheel() {
    setFlywheelOpenLoop(0);
  }

  public void setTurretPitchOpenLoop(double value) {
    io.setTurretPitchOpenLoop(value);
  }

  public void setLinearMm(double value) {
    io.setLinearActuatorPosition(value);
  }

  public void setTurretYawOpenLoop(double value) {
    io.setTurretYawOpenLoop(value);
  }

  // ccw positive
  public double getTurretYaw() {
    return inputs.turretYawRadians;
  }

  /** Points the turret toward the hub using odometry pose. */
  public void pointAtHub() {
    autoAim(FieldConstants.HUB.toTranslation2d());
  }

  public double getYawAtTime(double time) {
    return yawBuffer.getValueAtTimestamp(time);
  }

  public void autoAim(Translation2d itsPose) {
    autoAim(
        itsPose,
        Drive.getInstance().getPose(),
        Drive.getInstance().getChassisSpeeds(),
        false,
        false);
  }

  public void autoAim(Translation2d itsPose, Pose2d myPose, ChassisSpeeds chassisSpeeds) {
    autoAim(itsPose, myPose, chassisSpeeds, false, false);
  }

  public void autoAim(Translation2d itsPose, boolean flywheel) {
    autoAim(
        itsPose,
        Drive.getInstance().getPose(),
        Drive.getInstance().getChassisSpeeds(),
        flywheel,
        false);
  }

  public void autoAim(Translation2d itsPose, boolean flywheel, double distanceOffset) {
    autoAim(
        itsPose,
        Drive.getInstance().getPose(),
        Drive.getInstance().getChassisSpeeds(),
        flywheel,
        false,
        false,
        distanceOffset);
  }

  public void autoAim(
      Translation2d itsPose, Pose2d myPose, ChassisSpeeds chassisSpeeds, boolean flywheel) {
    autoAim(itsPose, myPose, chassisSpeeds, flywheel, false);
  }

  public void autoAimDry(Translation2d itsPose, Pose2d myPose, ChassisSpeeds chassisSpeeds) {
    autoAim(itsPose, myPose, chassisSpeeds, false, true);
  }

  public void autoAimDry(Translation2d itsPose) {
    autoAim(
        itsPose,
        Drive.getInstance().getPose(),
        Drive.getInstance().getChassisSpeeds(),
        false,
        true);
  }

  public void autoAim(
      Translation2d itsPose,
      Pose2d myPose,
      ChassisSpeeds chassisSpeeds,
      boolean flywheel,
      boolean dryRun) {
    autoAim(itsPose, myPose, chassisSpeeds, flywheel, dryRun, false);
  }

  public void autoAimYawOff(
      Translation2d itsPose,
      Pose2d myPose,
      ChassisSpeeds chassisSpeeds,
      boolean flywheel,
      boolean dryRun) {
    autoAim(itsPose, myPose, chassisSpeeds, flywheel, dryRun, true);
  }

  public void autoAim(
      Translation2d itsPose,
      Pose2d myPose,
      ChassisSpeeds chassisSpeeds,
      boolean flywheel,
      boolean dryRun,
      boolean yawDisabled) {
    autoAim(itsPose, myPose, chassisSpeeds, flywheel, dryRun, yawDisabled, 0);
  }

  // positive distanceoffset will create overshoot condition
  public void autoAim(
      Translation2d itsPose,
      Pose2d myPose,
      ChassisSpeeds chassisSpeeds,
      boolean flywheel,
      boolean dryRun,
      boolean yawDisabled,
      double distanceOffset) {
    Transform2d robotToShooter = new Transform2d(.12, 0, Rotation2d.kZero);

    double delaySeconds = 0.1;
    myPose = myPose.exp(chassisSpeeds.toTwist2d(delaySeconds));
    myPose = myPose.transformBy(robotToShooter);

    double distanceToHub = itsPose.getDistance(myPose.getTranslation());

    Logger.recordOutput("/Shooter/distanceToHub", distanceToHub);

    double shooterVx =
        chassisSpeeds.vxMetersPerSecond
            - (chassisSpeeds.omegaRadiansPerSecond * robotToShooter.getY());
    double shooterVy =
        chassisSpeeds.vyMetersPerSecond
            + (chassisSpeeds.omegaRadiansPerSecond * robotToShooter.getX());

    double angleToHub =
        itsPose.minus(myPose.getTranslation()).getAngle().minus(myPose.getRotation()).getRadians();

    double vrad = -shooterVx * Math.cos(angleToHub) - shooterVy * Math.sin(angleToHub);
    double vtan = shooterVx * Math.sin(angleToHub) - shooterVy * Math.cos(angleToHub);
    Logger.recordOutput("/Shooter/vrad", vrad);
    Logger.recordOutput("/Shooter/vtan", vtan);

    ShotParams shotParams =
        shotCalculator.calculateShotParams(
            itsPose.getDistance(myPose.getTranslation()) + distanceOffset, vrad, vtan);
    Logger.recordOutput("/Shooter/shotParams/yawOffsetRadians", shotParams.yawOffsetRadians());
    Logger.recordOutput(
        "/Shooter/shotParams/flywheelVelocityRotationsPerMinute",
        shotParams.flywheelVelocityRotationsPerMinute());
    Logger.recordOutput(
        "/Shooter/shotParams/linearActuatorExtensionMillimeters",
        shotParams.linearActuatorExtensionMillimeters());

    double angle = angleToHub + shotParams.yawOffsetRadians();
    double angleSetpoint = MathUtil.inputModulus(angle, yawModuloMax, yawModuloMin);
    Logger.recordOutput("/Shooter/angleSetpoint", angleSetpoint);

    if (!dryRun) {
      io.setLinearActuatorPosition(shotParams.linearActuatorExtensionMillimeters());
      if (!yawDisabled) {
        setTurretYaw(angleSetpoint);
      }
      if (flywheel) {
        setFlywheelVelocityRpm(shotParams.flywheelVelocityRotationsPerMinute());
      } else {
        stopFlywheel();
      }
    }
  }

  public void handleRecalibrationRequest(BooleanSupplier supplier) {
    if (supplier.getAsBoolean()) {
      recalibrateYaw();
    }
  }

  public Translation2d getShotTarget() {
    return getShotTarget(Drive.getInstance().getPose().getTranslation());
  }

  public Translation2d getShotTarget(Translation2d myPose) {
    // if we are within our side
    if (myPose.getX() < FieldConstants.HUB.getX()) {
      return FieldConstants.HUB.toTranslation2d();
    } else {
      if (myPose.getY() < FieldConstants.HUB.getY()) { // right
        return FieldConstants.SHOT_TARGET_R.toTranslation2d();
      } else {
        return FieldConstants.SHOT_TARGET_L.toTranslation2d();
      }
    }
  }

  public void scanForTarget() {
    double setpoint = getTurretYaw();
    setpoint += autoAimDirection;
    if (setpoint * Math.signum(autoAimDirection) > 1.57) {
      autoAimDirection = -autoAimDirection;
    }
    setTurretYaw(setpoint);
  }

  public void setTurretYaw(double setpoint) {
    if (isTurretInit) {
      io.setTurretYaw(setpoint);
    } else {
      io.setTurretYawOpenLoop(0);
    }
  }

  public Command autoAimCommandAuto() {
    return Commands.run(() -> autoAim(getShotTarget(), true), this);
  }

  public Command autoAimCommandAutoDryish() {
    return Commands.run(() -> autoAim(getShotTarget(), false), this);
  }

  public void set28TurretAngleSupplier(AbsoluteEncoder enc) {
    io.set28TurretAngleSupplier(enc);
  }

  private double calculateDoubleEncoderPosition() {
    return DoubleEncoder.processEncoderValues(inputs.position28Radians, inputs.position26Radians);
  }

  public void forceZeroYaw() {
    wasZeroForced = true;
    limpMode = false;
    io.zeroYaw();
    Vision.getInstance().uninit();
  }

  public void recalibrateYaw() {
    switch (Robot.VERSION) {
      case V1:
        io.zeroYaw();
        if (!isTurretInit) {
          isTurretInit = true;
        }
        break;
      case V2:
        recalibrateYawV2();
        break;
      default:
        throw new IllegalStateException("Invalid robot version");
    }
  }

  public void checkForLimpMode() {
    double sunGearVelocity26 = inputs.velocity26RadiansPerSecond * 26.0 / 200.0;
    double sunGearVelocity28 = inputs.velocity28RadiansPerSecond * 28.0 / 200.0;

    double avgVelocity = (sunGearVelocity26 + sunGearVelocity28) / 2;

    double difference = Math.abs(sunGearVelocity26 - sunGearVelocity28);

    double percentDifference = Math.abs(difference / avgVelocity);

    boolean shouldLimpMode = (difference > .02 && percentDifference > .2) || difference > .1;

    double currentTime = inputs.timestamp;

    if (!shouldLimpMode) {
      lastNonLimpTime = currentTime;
    }

    double timeInLimp = currentTime - lastNonLimpTime;

    if (shouldLimpMode && timeInLimp > .1) {
      isTurretInit = false;
      limpMode = true;
    }

    Logger.recordOutput("/Shooter/limpModeChecker/difference", difference);
    Logger.recordOutput("/Shooter/limpModeChecker/percentDifference", percentDifference);
    Logger.recordOutput("/Shooter/limpModeChecker/timeInLimp", timeInLimp);
  }

  public void recalibrateYawV2() {
    if (wasZeroForced) {
      System.out.println("[WARNING] Zero was previously forced; restart robot code to reenable...");
    }
    double newPosition;
    if (inputs.velocity26RadiansPerSecond > .01 || inputs.velocity28RadiansPerSecond > .01) {
      System.out.println("[WARNING] Recalibrating yaw failed (moving too quickly)");
      return;
    }
    if (inputs.encoder26Connected && inputs.encoder28Connected) {
      newPosition = calculateDoubleEncoderPosition();
    } else {
      System.out.println("[WARNING] Recalibrating yaw failed (no 26)");
      isTurretInit = false;
      return;
    }
    if (!Double.isFinite(newPosition)) {
      System.out.println("[WARNING] Recalibrating yaw failed (NaN)");
      isTurretInit = false;
      return;
    }
    double currentPos = inputs.turretYawRadians;
    if (isTurretInit && Math.abs(currentPos - newPosition) > .1) {
      isTurretInit = false;
      System.out.println("ENCODER DIFFERENCE TOO BIG! IT PROB SKIPPED A GEAR!");
    } else {
      isTurretInit = true;
      io.zeroYaw(newPosition);
    }
  }

  public boolean isLimpMode() {
    return limpMode;
  }

  public boolean isTurretInit() {
    return isTurretInit;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
    yawBuffer.add(inputs.turretYawRadians, inputs.timestamp);
    checkForLimpMode();

    if (!isTurretInit || DriverStation.isDisabled()) {
      recalibrateYaw();
    }

    if (isLimpMode()) {
      System.out.println("[WARNING] shooter in limp mode!");
    }
  }
}
