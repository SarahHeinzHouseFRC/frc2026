package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.PnpResult;

public class PhotonCameraIO implements CameraIO {
  private PhotonCamera camera;
  private String cameraName;
  private Transform3d cameraToRobot;
  private Transform3d robotToCamera;
  private Pose3d latestCameraPose = Pose3d.kZero;
  private boolean invert = true;

  @Override
  public void setIsBlue(boolean blue) {
    invert = !blue;
  }

  //  private StructPublisher<Pose3d> publisher;

  public PhotonCameraIO(String cameraName) {
    this(cameraName, Transform3d.kZero);
  }

  public PhotonCameraIO(String cameraName, Transform3d robotToCamera) {
    this.cameraToRobot = robotToCamera.inverse();
    this.robotToCamera = robotToCamera;
    this.cameraName = cameraName;
    camera = new PhotonCamera(cameraName);
    //    publisher =
    //        NetworkTableInstance.getDefault()
    //            .getStructTopic("/SHARP/Vision" + cameraName, Pose3d.struct)
    //            .publish();
  }

  @Override
  public void updateInputs(CameraIOInputs inputs) {
    List<PoseObservation> results = new ArrayList<>();
    double start = Timer.getFPGATimestamp();
    List<PhotonPipelineResult> photonResults = camera.getAllUnreadResults();
    if (Timer.getFPGATimestamp() - start > 1) {
      System.out.println("[WARNING] getAllUnreadResults took too long!");
    }

    double latestTimestamp = 0;

    int length = photonResults.size();
    int i = length - 10;
    if (i < 0) {
      i = 0;
    }

    for (; i < length; i++) {
      PhotonPipelineResult photonResult = photonResults.get(i);
      Optional<MultiTargetPNPResult> multiTagResult = photonResult.getMultiTagResult();
      if (multiTagResult.isPresent()) {
        PnpResult pnpResult = multiTagResult.get().estimatedPose;
        double targetDistance = 0;
        int targetCount = 0;
        for (PhotonTrackedTarget target : photonResult.targets) {
          if (target.fiducialId < 0) continue;
          targetCount++;
          targetDistance +=
              target.bestCameraToTarget.getTranslation().getDistance(Translation3d.kZero);
        }
        targetDistance /= targetCount;
        // Pose3d redPose = new Pose3d(new Translation3d(16.513, 8.043, 0), new Rotation3d(0,0,Math.PI));
        Pose3d redPose = new Pose3d(new Translation3d(16.541, 8.069, 0), new Rotation3d(0,0,Math.PI));
        Pose3d bluePose = Pose3d.kZero;
        Pose3d best = (invert ? redPose : bluePose).plus(pnpResult.best).plus(cameraToRobot);
        Pose3d alt = (invert ? redPose : bluePose).plus(pnpResult.best).plus(cameraToRobot);
        best = getBest(best, alt);
        results.add(
            new PoseObservation(
                photonResult.getTimestampSeconds(),
                best,
                pnpResult.ambiguity,
                pnpResult.bestReprojErr,
                targetCount,
                targetDistance));
        if (photonResult.getTimestampSeconds() > latestTimestamp) {
          latestTimestamp = photonResult.getTimestampSeconds();
          latestCameraPose = best.plus(robotToCamera);
        }
      } else if (!photonResult.targets.isEmpty()) {
        if (photonResult.targets.size() != 1) {
          System.out.println(
              "[WARNING] if there is no multitag there should be at most 1 tag present!");
        } else {
          PhotonTrackedTarget target = photonResult.targets.get(0);
          Optional<Pose3d> tagPose =
              VisionConstants.aprilTagFieldLayout.getTagPose(target.fiducialId);
          if (tagPose.isPresent()) {
            Pose3d best = tagPose.get().plus(target.bestCameraToTarget.inverse()).plus(cameraToRobot);
            Pose3d alt = tagPose.get().plus(target.altCameraToTarget.inverse()).plus(cameraToRobot);
            best = getBest(best, alt);
            results.add(
                new PoseObservation(
                    photonResult.getTimestampSeconds(),
                    best,
                    target.poseAmbiguity,
                    -1,
                    1,
                    target.bestCameraToTarget.getTranslation().getDistance(Translation3d.kZero)));
            if (photonResult.getTimestampSeconds() > latestTimestamp) {
              latestTimestamp = photonResult.getTimestampSeconds();
              latestCameraPose = best.plus(robotToCamera);
            }
          } else {
            //            System.out.println("[WARNING] tag pose not present!");
          }
        }
      }
    }
    inputs.results = results.toArray(new PoseObservation[0]);
    Logger.recordOutput("/SHARP/Vision" + cameraName, latestCameraPose);
  }

  private static Pose3d getBest(Pose3d best, Pose3d alternate) {
    double tiltBest =
        Math.acos(Math.cos(best.getRotation().getX()) * Math.cos(best.getRotation().getY()));
    double tiltAlternate =
        Math.acos(
            Math.cos(alternate.getRotation().getX()) * Math.cos(alternate.getRotation().getY()));
    if (tiltBest > .5 && tiltBest > tiltAlternate) return alternate;

    double groundDistanceBest = best.getZ();
    double groundDistanceAlternate = alternate.getZ();
    if (groundDistanceBest > .3 && groundDistanceBest > groundDistanceAlternate) return alternate;

    return best;
  }
}
