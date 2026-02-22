package frc.robot.vision;

import java.util.ArrayList;

import frc.robot.math.Matrix3d;

public class CameraCoefs {
  private int camId;
  private ArrayList<Double> camMat;
  private ArrayList<Double> distCoef;

  public CameraCoefs(int camId, ArrayList<Double> camMat, ArrayList<Double> distCoef) {
    this.camId = camId;
    this.camMat = camMat;
    this.distCoef = distCoef;    
  }

  public int getCamId() {
    return camId;
  }

  public ArrayList<Double> getCamMat() {
    return camMat;
  }

  public ArrayList<Double> getDistCoef() {
    return  distCoef;
  }
}