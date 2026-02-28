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
  private CameraIO turretCam = new PhotonCameraIO("turretCam", turretCamTransform);
  private CameraIOInputsAutoLogged turretCamInputs = new CameraIOInputsAutoLogged();
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
    }

    Drive drive = Drive.getInstance();
    Shooter shooter = Shooter.getInstance();
    if (shooter.isTurretInit()) {
      for (CameraIO.PoseObservation obs : turretCamInputs.results) {
        double stddev = 0.9;
        Pose2d pose =
            obs.pose()
                .toPose2d()
                .plus(
                    (new Transform2d(.12, 0, new Rotation2d(shooter.getYawAtTime(obs.timestamp())))
                        .inverse()))
                .plus(new Transform2d(0d, 0d, Rotation2d.kPi));
        if (!isVisionInit) {
          double start = Timer.getFPGATimestamp();
          drive.setPose(pose);
          System.out.println("vision init took " + (Timer.getFPGATimestamp() - start));
          isVisionInit = true;
        }
        drive.addVisionMeasurement(pose, obs.timestamp(), VecBuilder.fill(stddev, stddev, stddev));
      }
    }
  }
}
