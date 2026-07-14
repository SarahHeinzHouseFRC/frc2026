package frc.robot.subsystems.ball;

import frc.robot.RobotContainer;
import frc.robot.TeleopShooter;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.launcher.LauncherSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.ball.BallConstants.*;

public class TeleopBallControl extends BallControl {
  public TeleopBallControl(BallSubsystem ball, DoubleSupplier intakeRequestSupplier, DoubleSupplier shootRequestSupplier) {
    this(ball, intakeRequestSupplier, shootRequestSupplier, TeleopBallControl::readyToShoot);
  }

  public TeleopBallControl(BallSubsystem ball, DoubleSupplier intakeRequestSupplier, DoubleSupplier shootRequestSupplier, BooleanSupplier readyToShootSupplier) {
    super(
        ball,
        () -> {
          double intakeRequest = intakeRequestSupplier.getAsDouble();
          double shootRequest = shootRequestSupplier.getAsDouble();
          boolean wantsIntake = Math.abs(intakeRequest) > 0.1;
          boolean wantsShoot = shootRequest > 0.1;

          double intakeSpeed = 0;
          double beltSpeed = 0;
          if (wantsIntake) {
            intakeSpeed =  intakeSpeedIntaking * intakeRequest;
            beltSpeed = beltSpeedIntaking * intakeRequest;
          } else if (wantsShoot) {
            intakeSpeed =  intakeSpeedShooting;
            beltSpeed = beltSpeedShooting;
          }


          boolean readyToShoot = readyToShootSupplier.getAsBoolean();

          double indexerSpeed = 0;
          if (wantsShoot && readyToShoot) {
            indexerSpeed = indexerSpeedShooting;
          } else if (wantsIntake) {
            indexerSpeed = indexerSpeedIntaking * Math.abs(intakeRequest);
          } else if (wantsShoot) {
            indexerSpeed = indexerSpeedPreShooting;
          }

          return new BallInputs(intakeSpeed, beltSpeed, indexerSpeed);
        }
    );
  }

  public static boolean readyToShoot() {
    boolean flywheelReady = LauncherSubsystem.getInstance().isFlywheelAtSpeed();
    boolean aimReady = switch (TeleopShooter.getInstance().getShooterMode()) {
      case DRIVE_AUTO -> Drive.getInstance().isYawAtSetpoint();
      case TURRET_AUTO -> TurretSubsystem.getInstance().isPanAtSetpoint();
      case MANUAL -> true;
      default -> false;
    };

    return flywheelReady && aimReady;
  }

}
