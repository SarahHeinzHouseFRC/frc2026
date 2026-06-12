package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drive.Drive;
import frc.robot.utils.Utils;

public class ShotCalculator {
  private final double[] rpmLut = new double[] {2900, 3047, 3207, 3367, 3489, 3610, 3746, 4150};

  private final double[] linearLut = new double[] {0, 0, 0, 0, 5, 10, 15, 25};

  private final double[] timeLut = new double[] {1.33, 1.62, 1.82, 1.86, 1.87, 1.88, 1.90, 1.99, 2.00};

  private final double timeDelay = 0.0;

  private final double lutStart = 1.5;

  private final double lutStep = 0.5;

  private static final ShotCalculator instance = new ShotCalculator();

  private ShotParams shotParams = new ShotParams(0, 0, 0);

  private double shotAngle = 0;

  private ShotCalculator() {}

  public static ShotCalculator getInstance() {
    return instance;
  }

  public void update() {
    update(Drive.getInstance().getPose(), Drive.getInstance().getChassisSpeeds());
  }

  private Translation2d getTarget(Pose2d myPose) {
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

  public void update(Pose2d myPose, ChassisSpeeds chassisSpeeds) {
    Translation2d itsPose = getTarget(myPose);
    Transform2d robotToShooter = new Transform2d(.12, 0, Rotation2d.kZero);

    double delaySeconds = 0.1;
//    myPose = myPose.exp(chassisSpeeds.toTwist2d(delaySeconds));
    myPose = myPose.transformBy(robotToShooter);

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

    shotParams =
        calculateShotParams(
            itsPose.getDistance(myPose.getTranslation()), vrad, vtan);

    shotAngle = angleToHub + shotParams.yawOffsetRadians();
  }

  private ShotParams calculateShotParams(double distanceMeters, double velocityRadialMetersPerSecond, double velocityTangentialMetersPerSecond) {
    double distanceRadial = distanceMeters;
    double distanceTangential = 0;
    double rpm = 0;
    double linear = 15;
    double iMax = 2;
    double lastTime = 0;
    for (int i = 0; i < iMax; i++) {
      double distance = Math.hypot(distanceRadial, distanceTangential);

      rpm = Utils.lutLerp(rpmLut, lutStart, lutStep, distance);
      linear = Utils.lutLerp(linearLut, lutStart, lutStep, distance);
      if (timeLut == null) {
        System.out.println("[WARNING] timeLut is null");
        break;
      }
      double time = Utils.lutLerp(timeLut, lutStart, lutStep, distance) + timeDelay;
      boolean converged = Math.abs(lastTime - time) < .1;
      if (converged) {
        break;
      } else if (i == iMax - 1) {
//        System.out.println("[WARNING] iterativeShotCalculator did not converge");
      }
      lastTime = time;

      // move robot and calc new params
      distanceRadial = distanceMeters + velocityRadialMetersPerSecond * time;
      distanceTangential = velocityTangentialMetersPerSecond * time;
    }
    return new ShotParams(rpm, linear, Math.atan2(distanceTangential, distanceRadial));
  };

  public ShotParams getShotParams() {
    return shotParams;
  }

  public double getShotAngle() {
    return shotAngle;
  }
}
