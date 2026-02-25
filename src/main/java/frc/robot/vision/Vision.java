package frc.robot.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.drive.Drive;
import frc.robot.shooter.Shooter;
import java.io.IOException;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final Transform3d turretCamTransform =
      new Transform3d(.18, 0, .5, new Rotation3d(0, -Math.PI / 6, 0));
  // private CameraIO turretCam =
  //     new PhotonCameraIO("camera-4", turretCamTransform);
  // private CameraIOInputsAutoLogged turretCamInputs = new CameraIOInputsAutoLogged();

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
    // load bearing System.out.println btw. dont remove.
    // forces the jvm to actually load the apriltag layout
    // rather than waiting for first use (camera sees tag)
    System.out.println(
        "Using apriltag layout with "
            + VisionConstants.aprilTagFieldLayout.getTags().size()
            + " tags");

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
    // turretCam.updateInputs(turretCamInputs);
    // Logger.processInputs("Vision/Turret Cam", turretCamInputs);

    // Drive drive = Drive.getInstance();
    // Shooter shooter = Shooter.getInstance();
    // for (CameraIO.PoseObservation obs : turretCamInputs.results) {
    //   double stddev = 0.9;
    //   drive.addVisionMeasurement(
    //       obs.pose()
    //           .toPose2d()
    //           .plus(
    //               (new Transform2d(.12, 0, new Rotation2d(shooter.getYawAtTime(obs.timestamp())))
    //                   .inverse()))
    //           .plus(new Transform2d(0d, 0d, Rotation2d.kPi)),
    //       obs.timestamp(),
    //       VecBuilder.fill(stddev, stddev, stddev));
    // }
  }
}
