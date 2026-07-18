package frc.robot.subsystems.ball;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;
import frc.robot.TeleopShooter;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.launcher.LauncherSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.ball.BallConstants.*;

public class TeleopBallControl extends BallControl {
  public static final double INDEXER_UNJAM_TIME = 1.0;
  public static final double INDEXER_UNJAM_POWER = -0.5;
  private double jamDetectedTime = -1;


  public TeleopBallControl(BallSubsystem ball, DoubleSupplier intakeRequestSupplier, DoubleSupplier shootRequestSupplier, BooleanSupplier unjamRequestSupplier) {
    this(ball, intakeRequestSupplier, shootRequestSupplier, TeleopBallControl::readyToShoot, unjamRequestSupplier);
  }

  public TeleopBallControl(BallSubsystem ball, DoubleSupplier intakeRequestSupplier, DoubleSupplier shootRequestSupplier, BooleanSupplier readyToShootSupplier, BooleanSupplier unjamRequestSupplier) {
    super(
        ball,
        () -> BallInputs.ZERO
    );
    setBallInputsSupplier(() -> calculateBallInputs(
        intakeRequestSupplier.getAsDouble(),
        shootRequestSupplier.getAsDouble(),
        readyToShootSupplier.getAsBoolean(),
        unjamRequestSupplier.getAsBoolean()
    ));
  }

  private BallInputs calculateBallInputs(double intakeRequest, double shootRequest, boolean readyToShoot, boolean unjamRequest) {
    if (jamDetectedTime < 0 && BallSubsystem.getInstance().isIndexerJammed()) {
      jamDetectedTime = Timer.getFPGATimestamp();
    } else if (!BallSubsystem.getInstance().isIndexerJammed()) {
      jamDetectedTime = -1;
    }

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

    double indexerSpeed = 0;
    if (wantsShoot && readyToShoot) {
      indexerSpeed = indexerSpeedShooting;
    } else if (wantsIntake) {
      indexerSpeed = indexerSpeedIntaking * Math.abs(intakeRequest);
    } else if (wantsShoot) {
      indexerSpeed = indexerSpeedPreShooting;
    }

    if (Timer.getFPGATimestamp() - jamDetectedTime < INDEXER_UNJAM_TIME || unjamRequest) {
      indexerSpeed = INDEXER_UNJAM_POWER;
      beltSpeed = INDEXER_UNJAM_POWER;
    }

    return new BallInputs(intakeSpeed, beltSpeed, indexerSpeed);
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
