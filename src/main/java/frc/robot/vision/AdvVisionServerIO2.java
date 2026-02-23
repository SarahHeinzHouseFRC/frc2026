package frc.robot.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.drive.Drive;
import frc.robot.protos.DataHandling;
import frc.robot.protos.DataHandling.CameraCoefficients;
import frc.robot.protos.DataHandling.CameraPosition;
import frc.robot.protos.DataHandling.ClientRequest;
import frc.robot.protos.DataHandling.ServerRequest;
import frc.robot.protos.DataHandling.SetCameraCoefficients;
import frc.robot.protos.DataHandling.TagInCamCoords;
import frc.robot.protos.VisionSystemGrpc.VisionSystemImplBase;
import io.grpc.Server;
import io.grpc.ServerBuilder;
import io.grpc.stub.StreamObserver;
import java.io.IOException;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicBoolean;

public class AdvVisionServerIO2 {

  private static final int PORT = 50001;

  private Server server;
  private List<CameraCoefs> cameras = Arrays.asList(CameraCoefs.getDefaultCameraCoefs(2));

  private AtomicBoolean isInitialized = new AtomicBoolean(false);

  private StructPublisher<Pose2d> publisher;

  public AdvVisionServerIO2() {
    server = ServerBuilder.forPort(PORT).addService(new VisionSystemImpl()).build();

    publisher =
        NetworkTableInstance.getDefault()
            .getStructTopic("/SHARP/Vision/AdvVisionServerIO2", Pose2d.struct)
            .publish();
  }

  public void start() throws IOException {
    server.start();
  }

  private class VisionSystemImpl extends VisionSystemImplBase {

    public SetCameraCoefficients buildSetCameraCoefficientsRequest(CameraCoefs cameraCoefs) {
      SetCameraCoefficients.Builder out = SetCameraCoefficients.newBuilder();
      out.setCameraId(cameraCoefs.getCamId());
      CameraCoefficients.Builder coefs = CameraCoefficients.newBuilder();
      coefs.addAllCameraMatrix(cameraCoefs.getCamMat());
      coefs.addAllDistortionCoefficients(cameraCoefs.getDistCoef());
      out.setCameraCoefficients(coefs.build());
      return out.build();
    }

    public StreamObserver<DataHandling.ClientRequest> openControlStream(
        StreamObserver<DataHandling.ServerRequest> responseObserver) {
      for (CameraCoefs e : cameras) {
        ServerRequest setCameraCoefficients =
            ServerRequest.newBuilder()
                .setSetCameraCoefficients(buildSetCameraCoefficientsRequest(e))
                .build();
        try {
          responseObserver.onNext(setCameraCoefficients);
        } catch (Exception exc) {
          exc.printStackTrace();
          throw exc;
        }
      }
      return new StreamObserver<DataHandling.ClientRequest>() {

        @Override
        public void onNext(ClientRequest value) {
          switch (value.getMsgCase()) {
            case REPORT_CAMERA_POSITIONS:
              reportCameraPositions(value);
              break;
            default:
              System.err.println("Unknown message: " + value.toString());
              break;
          }
        }

        @Override
        public void onError(Throwable t) {
          t.printStackTrace();
        }

        @Override
        public void onCompleted() {
          System.out.println("Camera client disconnected");
          responseObserver.onCompleted();
        }
      };
    }
  }

  private Transform3d getTransformFromArray(List<Double> data) {
    double[] d = new double[data.size()];
    for (int i = 0; i < data.size(); ++i) {
      d[i] = data.get(i);
    }
    return new Transform3d(new Matrix<N4, N4>(Nat.N4(), Nat.N4(), d));
  }

  private Vector<N3> getStdDevVector(double distance) {
    // TODO compute std dev based on ambiguity and distance
    double translationStdDev = 0.1 * distance * distance;
    return VecBuilder.fill(translationStdDev, translationStdDev, 0.5);
  }

  private void reportCameraPositions(ClientRequest req) {
    Drive drive = Drive.getInstance();

    List<CameraPosition> pos = req.getReportCameraPositions().getCameraPositionList();
    for (CameraPosition e : pos) {
      int camId = e.getCameraId();
      List<TagInCamCoords> tags = e.getTagInCamCoordsList();
      for (TagInCamCoords a : tags) {
        int tagId = a.getTagId();
        Optional<Pose3d> tagPose = VisionConstants.aprilTagFieldLayout.getTagPose(tagId);

        if (!tagPose.isPresent()) continue;

        double errorBest = a.getBestReprojectionError();
        double errorWorse = a.getWorseReprojectionError();

        double ambiguity = errorBest / errorWorse;

        if (ambiguity < 0.2) {
          Transform3d bestCameraToTarget = getTransformFromArray(a.getNwuBestPoseList());
          double distance = bestCameraToTarget.getTranslation().getNorm();
          // System.out.println("bestCameraToTarget: "+bestCameraToTarget);
          Pose3d poseBest = tagPose.get().transformBy(bestCameraToTarget.inverse());
          // TODO add .transformBy(robotToCamera.inverse())

          if (isInitialized.compareAndSet(false, true)) {
            drive.resetVisionMeasurement(poseBest.toPose2d());
          } else {
            double timestamp_seconds = Timer.getFPGATimestamp() - (a.getLatencyUs() * 1e-6);
            System.out.println(
                "addVisionMeasurement: "
                    + poseBest.toPose2d()
                    + " timestamp(sec): "
                    + timestamp_seconds
                    + " ambiguity: "
                    + ambiguity);
            drive.addVisionMeasurement(
                poseBest.toPose2d(), timestamp_seconds, getStdDevVector(distance));
            System.out.println("Drive pose (requires odometry working): " + drive.getPose());
          }
          publisher.set(poseBest.toPose2d());
        }
      }
    }
  }
}
