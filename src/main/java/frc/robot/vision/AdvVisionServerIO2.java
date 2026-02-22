package frc.robot.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;
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
import java.util.ArrayList;
import java.util.List;

public class AdvVisionServerIO2 {

  private static final int PORT = 50001;

  private Server server;
  private ArrayList<CameraCoefs> cameras;

  public AdvVisionServerIO2() {
    server = ServerBuilder.forPort(PORT).addService(new VisionSystemImpl()).build();
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

  private Pose3d getPoseFromArray(List<Double> data) {
    double[] d = new double[data.size()];
    for (int i = 0; i < data.size(); ++i) {
      d[i] = data.get(i);
    }
    return new Pose3d(new Matrix<N4, N4>(Nat.N4(), Nat.N4(), d));
  }

  private void reportCameraPositions(ClientRequest req) {
    ArrayList<Pose3d> validPoses = new ArrayList<>();

    List<CameraPosition> pos = req.getReportCameraPositions().getCameraPositionList();
    for (CameraPosition e : pos) {
      int camId = e.getCameraId();
      List<TagInCamCoords> tags = e.getTagInCamCoordsList();
      for (TagInCamCoords a : tags) {
        int tagId = a.getTagId();
        double errorBest = a.getBestReprojectionError();
        double errorWorse = a.getWorseReprojectionError();

        double ambiguity = errorBest / errorWorse;

        if (ambiguity < 0.2) {

          Pose3d poseBest = getPoseFromArray(a.getMat1List());
          // Pose3d poseWorse = getPoseFromArray(a.getMat2List());

          validPoses.add(poseBest);
        }
      }
    }

    // Implement Math

    Drive drive = Drive.getInstance();
    drive.addVisionMeasurement(
        validPoses.get(0).toPose2d(),
        0,
        new Matrix<N3, N1>(Nat.N3(), Nat.N1(), new double[] {0, 0, 0}));
  }
}
