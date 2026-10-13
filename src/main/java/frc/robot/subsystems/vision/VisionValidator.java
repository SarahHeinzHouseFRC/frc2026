package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.drive.Drive;
import java.util.Optional;

public final class VisionValidator {
  private VisionValidator() {}

  public static Pose3d chooseBestPose(Pose3d best, Pose3d alternate) {
    return poseScore(alternate) < poseScore(best) ? alternate : best;
  }

  static double poseScore(Pose3d pose) {
    if (!isFinite(pose)) {
      return Double.POSITIVE_INFINITY;
    }

    double margin = VisionConstants.FIELD_BORDER_MARGIN_METERS;
    double score = 0.0;
    score +=
        distanceOutside(
            pose.getX(),
            -margin,
            VisionConstants.aprilTagFieldLayout.getFieldLength() + margin);
    score +=
        distanceOutside(
            pose.getY(),
            -margin,
            VisionConstants.aprilTagFieldLayout.getFieldWidth() + margin);
    score += distanceOutside(pose.getZ(), VisionConstants.MIN_Z, VisionConstants.MAX_Z);
    score += Math.max(0.0, getTilt(pose) - VisionConstants.MAX_TILT_RADIANS);
    return score;
  }

  public static boolean shouldRejectPose(
      CameraIO.PoseObservation observation,
      Pose2d pose,
      Drive drive,
      boolean visionInitialized) {
    Pose3d pose3d = observation.pose();
    double now = Timer.getFPGATimestamp();

    if (observation.tagCount() <= 0
        || !Double.isFinite(observation.averageTagDistance())
        || observation.averageTagDistance() <= 0.0
        || !Double.isFinite(observation.timestamp())
        || observation.timestamp() <= 0.0
        || now - observation.timestamp() > VisionConstants.MAX_OBSERVATION_AGE_SECONDS
        || observation.timestamp() - now > VisionConstants.MAX_FUTURE_TIMESTAMP_SECONDS
        || !isPoseValid(pose3d, pose)) {
      return true;
    }

    if (observation.tagCount() == 1) {
      if (!Double.isFinite(observation.ambiguity())
          || observation.ambiguity() < 0.0
          || observation.ambiguity() > VisionConstants.MAX_SINGLE_TAG_AMBIGUITY) {
        return true;
      }

      if (visionInitialized) {
        Optional<Pose2d> estimatedPoseAtCapture = drive.samplePoseAt(observation.timestamp());
        Pose2d poseForComparison = estimatedPoseAtCapture.orElseGet(drive::getPose);
        double translationError =
            pose.getTranslation().getDistance(poseForComparison.getTranslation());
        if (translationError > VisionConstants.MAX_SINGLE_TAG_POSE_DIFFERENCE_METERS) {
          return true;
        }
      }
    } else if (!Double.isFinite(observation.reprojError())
        || observation.reprojError() < 0.0
        || observation.reprojError() > VisionConstants.MAX_MULTI_TAG_REPROJECTION_ERROR) {
      return true;
    }

    return false;
  }

  private static boolean isPoseValid(Pose3d pose3d, Pose2d pose) {
    if (!isFinite(pose3d)) {
      return false;
    }

    double margin = VisionConstants.FIELD_BORDER_MARGIN_METERS;
    return pose.getX() >= -margin
        && pose.getX() <= VisionConstants.aprilTagFieldLayout.getFieldLength() + margin
        && pose.getY() >= -margin
        && pose.getY() <= VisionConstants.aprilTagFieldLayout.getFieldWidth() + margin
        && pose3d.getZ() >= VisionConstants.MIN_Z
        && pose3d.getZ() <= VisionConstants.MAX_Z
        && getTilt(pose3d) <= VisionConstants.MAX_TILT_RADIANS;
  }

  private static boolean isFinite(Pose3d pose) {
    return Double.isFinite(pose.getX())
        && Double.isFinite(pose.getY())
        && Double.isFinite(pose.getZ())
        && Double.isFinite(pose.getRotation().getX())
        && Double.isFinite(pose.getRotation().getY())
        && Double.isFinite(pose.getRotation().getZ());
  }

  private static double getTilt(Pose3d pose) {
    double cosine =
        Math.cos(pose.getRotation().getX()) * Math.cos(pose.getRotation().getY());
    return Math.acos(Math.max(-1.0, Math.min(1.0, cosine)));
  }

  private static double distanceOutside(double value, double minimum, double maximum) {
    if (value < minimum) {
      return minimum - value;
    }
    if (value > maximum) {
      return value - maximum;
    }
    return 0.0;
  }
}
