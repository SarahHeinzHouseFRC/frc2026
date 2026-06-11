package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;

import java.io.IOException;
import java.util.Optional;

public class Vision extends SubsystemBase {
//  private final Transform3d turretCamTransform =
//      new Transform3d(.18, 0, .5, new Rotation3d(0, -Math.PI / 6, 0));
  private final Transform3d frontLeftSwerveTransform =
      new Transform3d(.2921, .2921, .2344, new Rotation3d(0, -0.43633, 0.79037));
  private final Transform3d frontRightSwerveTransform =
      new Transform3d(0, 0, 0, new Rotation3d(0, 0, -Math.PI / 2)).plus(frontLeftSwerveTransform);
  private final Transform3d backLeftSwerveTransform =
      new Transform3d(0, 0, 0, new Rotation3d(0, 0, Math.PI / 2)).plus(frontLeftSwerveTransform);
  private final Transform3d backRightSwerveTransform =
      new Transform3d(0, 0, 0, new Rotation3d(0, 0, Math.PI)).plus(frontLeftSwerveTransform);
//  private CameraIO turretCam = new PhotonCameraIO("turretCam", turretCamTransform);
//  private CameraIO.CameraIOInputs turretCamInputs = new CameraIOInputsAutoLogged();

  private CameraIO leftCam = new PhotonCameraIO("leftCam", frontLeftSwerveTransform);
  private CameraIO.CameraIOInputs leftCamInputs = new CameraIO.CameraIOInputs();

  private CameraIO rightCam = new PhotonCameraIO("rightCam", frontRightSwerveTransform);
  private CameraIO.CameraIOInputs rightCamInputs = new CameraIO.CameraIOInputs();
  private boolean isBlue = true;

  private boolean isVisionInit = false;

  private static final Vision instance = new Vision();

  public static Vision getInstance() {
    return instance;
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
  }

  public boolean isVisionInit() {
    return isVisionInit;
  }

  @Override
  public void periodic() {
    leftCam.updateInputs(leftCamInputs);
    rightCam.updateInputs(rightCamInputs);

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
      leftCam.setIsBlue(isBlue);
      rightCam.setIsBlue(isBlue);
    }

      for (CameraIO.PoseObservation obs : leftCamInputs.results) {
        processPose(obs);
      }
      for (CameraIO.PoseObservation obs : rightCamInputs.results) {
        processPose(obs);
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
