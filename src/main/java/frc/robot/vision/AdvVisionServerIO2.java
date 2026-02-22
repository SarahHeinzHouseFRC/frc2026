package frc.robot.vision;

import io.grpc.Server;
import io.grpc.ServerBuilder;
import io.grpc.stub.StreamObserver;
import frc.robot.protos.VisionSystemGrpc.VisionSystemImplBase;

import java.util.ArrayList;

import frc.robot.drive.Drive;
import frc.robot.protos.DataHandling;
import frc.robot.protos.DataHandling.CameraCoefficients;
import frc.robot.protos.DataHandling.ClientRequest;
import frc.robot.protos.DataHandling.ServerRequest;
import frc.robot.protos.DataHandling.SetCameraCoefficients;

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

    public StreamObserver<DataHandling.ClientRequest> openControlStream(StreamObserver<DataHandling.ServerRequest> responseObserver) {
      for (CameraCoefs e : cameras) {
        ServerRequest setCameraCoefficients = ServerRequest.newBuilder().setSetCameraCoefficients(buildSetCameraCoefficientsRequest(e));
        try {
          responseObserver.onNext(setCameraCoefficients);
        } catch (Exception e) {
          e.printStackTrace();
          throw e;
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
              System.err.println("Unknown message: "+value.toString());
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

  private void reportCameraPositions(ClientRequest req) {
    // TODO
    double ambiguity = bestError / worseError;
    Drive drive = Drive.getInstance();
    if (ambiguity < 0.2) {
      drive.addVisionMeasurement(/* best position */);
    }
  }
}
