package frc.robot.vision;

import java.util.Arrays;
import java.util.List;

public class CameraCoefs {
  private int camId;
  private List<Double> camMat;
  private List<Double> distCoef;

  public CameraCoefs(int camId, List<Double> camMat, List<Double> distCoef) {
    this.camId = camId;
    this.camMat = camMat;
    this.distCoef = distCoef;
  }

  public int getCamId() {
    return camId;
  }

  public List<Double> getCamMat() {
    return camMat;
  }

  public List<Double> getDistCoef() {
    return distCoef;
  }

  public static CameraCoefs getDefaultCameraCoefs(int camId) {
    return new CameraCoefs(camId, Arrays.<Double>asList(
      922.9237643349662, 0.0, 647.4788707248217,
      0.0, 919.8858077883355, 399.87229770900007,
      0.0, 0.0, 1.0
    ), Arrays.<Double>asList(
      0.05877060800641068, 
      -0.10744454024922885,
      3.2631843588809995E-4,
      0.0014886030530316236,
      0.04142037702520378,
      6.850399028674193E-4,
      0.0014547294832255185,
      -0.0037595003107658287
    ));
  }
}
