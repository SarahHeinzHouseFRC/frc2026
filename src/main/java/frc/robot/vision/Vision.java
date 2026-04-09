package frc.robot.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.drive.Drive;
import frc.robot.shooter.Shooter;
import java.io.IOException;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final Transform3d turretCamTransform =
      new Transform3d(.18, 0, .5, new Rotation3d(0, -Math.PI / 6, 0));
  private final Transform3d frontLeftSwerveTransform =
      new Transform3d(.2921, .2921, .2344, new Rotation3d(0, -0.43633, 0.79037));
  private final Transform3d frontRightSwerveTransform =
      new Transform3d(0, 0, 0, new Rotation3d(0, 0, -Math.PI / 2)).plus(frontLeftSwerveTransform);
  private final Transform3d backLeftSwerveTransform =
      new Transform3d(0, 0, 0, new Rotation3d(0, 0, Math.PI / 2)).plus(frontLeftSwerveTransform);
  private final Transform3d backRightSwerveTransform =
      new Transform3d(0, 0, 0, new Rotation3d(0, 0, Math.PI)).plus(frontLeftSwerveTransform);
  private CameraIO turretCam = new PhotonCameraIO("turretCam", turretCamTransform);
  private CameraIOInputsAutoLogged turretCamInputs = new CameraIOInputsAutoLogged();

  private CameraIO leftCam = new PhotonCameraIO("leftCam", frontLeftSwerveTransform);
  private CameraIOInputsAutoLogged leftCamInputs = new CameraIOInputsAutoLogged();

  private CameraIO rightCam = new PhotonCameraIO("rightCam", frontRightSwerveTransform);
  private CameraIOInputsAutoLogged rightCamInputs = new CameraIOInputsAutoLogged();
  @AutoLogOutput private boolean isBlue = true;

  // VisionSystem handles updates from coprocessors and calls drive.addVisionMeasurement directly.
  private AdvVisionServerIO2 visionSystem = new AdvVisionServerIO2();

  private boolean isVisionInit = false;

  private static Vision instance;

  public static void init() {
    instance = new Vision();
  }

  public static Vision getInstance() {
    return instance;
  }

  public void uninit() {
    isVisionInit = false;
  }

  private Vision() {
    Optional<DriverStation.Alliance> allianceOptional = DriverStation.getAlliance();
    if (allianceOptional.isPresent() && allianceOptional.get() == DriverStation.Alliance.Red) {
      isBlue = false;
    }
    VisionConstants.aprilTagFieldLayout.setOrigin(
        isBlue
            ? AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide
            : AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide);
    turretCam.setIsBlue(isBlue);
    leftCam.setIsBlue(isBlue);
    rightCam.setIsBlue(isBlue);

    // load bearing System.out.println btw. dont remove.
    // forces the jvm to actually load the apriltag layout
    // rather than waiting for first use (camera sees tag)
    System.out.println(
        "Using apriltag layout with "
            + VisionConstants.aprilTagFieldLayout.getTags().size()
            + " tags on "
            + (isBlue ? "blue" : "red")
            + " side.");

    System.out.println("Starting vision server");
    try {
      visionSystem.start();
    } catch (IOException e) {
      e.printStackTrace();
      throw new RuntimeException("Cannot start vision server", e);
    }
  }

  @AutoLogOutput
  public boolean isVisionInit() {
    return isVisionInit;
  }

  @Override
  public void periodic() {
    turretCam.updateInputs(turretCamInputs);
    Logger.processInputs("Vision/Turret Cam", turretCamInputs);
    leftCam.updateInputs(leftCamInputs);
    Logger.processInputs("Vision/Left Cam", leftCamInputs);
    rightCam.updateInputs(rightCamInputs);
    Logger.processInputs("Vision/Right Cam", rightCamInputs);

    boolean localIsBlue = true;
    Optional<DriverStation.Alliance> allianceOptional = DriverStation.getAlliance();
    if (allianceOptional.isPresent() && allianceOptional.get() == DriverStation.Alliance.Red) {
      localIsBlue = false;
    }
    if (localIsBlue != isBlue) {
      isBlue = localIsBlue;
      isVisionInit = false;
      VisionConstants.aprilTagFieldLayout.setOrigin(
          isBlue
              ? AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide
              : AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide);
      turretCam.setIsBlue(isBlue);
      leftCam.setIsBlue(isBlue);
      rightCam.setIsBlue(isBlue);
    }

    Shooter shooter = Shooter.getInstance();
    if (shooter.isTurretInit()) {
      for (CameraIO.PoseObservation obs : turretCamInputs.results) {
        processPose(
            obs,
            new Transform2d(.12, 0, new Rotation2d(shooter.getYawAtTime(obs.timestamp())))
                .inverse());
      }
      for (CameraIO.PoseObservation obs : leftCamInputs.results) {
        processPose(obs);
      }
      for (CameraIO.PoseObservation obs : rightCamInputs.results) {
        processPose(obs);
      }
    }
  }

  private void processPose(CameraIO.PoseObservation obs) {
    processPose(obs, Transform2d.kZero);
  }

  private void processPose(CameraIO.PoseObservation obs, Transform2d transform) {
    Pose2d pose = obs.pose().toPose2d().plus(transform);
    Drive drive = Drive.getInstance();
    if (!isVisionInit) {
      double start = Timer.getFPGATimestamp();
      drive.setPose(pose);
      System.out.println("vision init took " + (Timer.getFPGATimestamp() - start));
      isVisionInit = true;
    }
    double stddev = Math.pow(obs.averageTagDistance(), 1.0 / obs.tagCount());
    drive.addVisionMeasurement(pose, obs.timestamp(), VecBuilder.fill(stddev, stddev, stddev * 5));
  }
}
