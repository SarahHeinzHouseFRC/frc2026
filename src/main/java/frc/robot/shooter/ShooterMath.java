package frc.robot.shooter;

import org.apache.commons.math3.ode.FirstOrderDifferentialEquations;
import org.apache.commons.math3.ode.FirstOrderIntegrator;
import org.apache.commons.math3.ode.events.EventHandler;
import org.apache.commons.math3.ode.nonstiff.DormandPrince853Integrator;
import org.apache.commons.math3.analysis.MultivariateVectorFunction;
import org.apache.commons.math3.fitting.leastsquares.EvaluationRmsChecker;
import org.apache.commons.math3.fitting.leastsquares.LeastSquaresFactory;
import org.apache.commons.math3.fitting.leastsquares.LeastSquaresOptimizer;
import org.apache.commons.math3.fitting.leastsquares.LeastSquaresProblem;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.fitting.leastsquares.LevenbergMarquardtOptimizer;

import frc.robot.math.Matrix3d;
import frc.robot.math.Vector3d;
import frc.robot.math.Transformation;

public class ShooterMath {
  Vector3d target;
  Transformation robotPosition;
  Vector3d robotVelocity;
  double theta;

  public ShooterMath(Vector3d target, Transformation robotPosition, Vector3d robotVelocity, double theta) {
    this.target = target;
    this.robotPosition = robotPosition;
    this.robotVelocity = robotVelocity;
    this.theta = theta;
  }
  
  public static class TargetEventHandler implements EventHandler{
    private final Vector3d target;

    public TargetEventHandler(Vector3d target) {
      this.target = new Vector3d(target);
    }

    @Override
    public double g(double t, double[] y) {
      return y[2]-target.z();
    }

    @Override
    public Action eventOccurred(double t, double[] y, boolean increasing) {
      if (increasing) {
        return Action.CONTINUE;
      } else {
        return Action.STOP;
      }
    }

    @Override
    public void init(double t0, double[] y0, double t) {}

    @Override
    public void resetState(double t, double[] y) {}

  }

  public static class BallPositionSolver implements FirstOrderDifferentialEquations {

    @Override
    public int getDimension() {
      return 6;
    }

    @Override
    public void computeDerivatives(double t, double[] y, double[] yDot) {
      // Gravitational force in m/s
      final double g = -9.81;
      // Ball mass
      final double m = 0.203;
      // Assuming Cd=0.47
      final double c = 0.00509;

      // Ball velocity
      final double v = Math.sqrt(y[3]*y[3]+y[4]*y[4]+y[5]*y[5]);

      // Forces on ball
      yDot[3] = -c*v*y[3]/m;
      yDot[4] = -c*v*y[4]/m;
      yDot[5] = g-c*v*y[5]/m;

      // Set derivatives of postiion
      yDot[0] = y[3];
      yDot[1] = y[4];
      yDot[2] = y[5];

    }
  }

  public static class Optimizer implements MultivariateVectorFunction {
    private final Vector3d target;
    private final Vector3d robotPosition;
    private final Vector3d robotVelocity;
    private final Matrix3d robotRotation;
    private final double theta;

    public Optimizer(Vector3d target, Vector3d robotPosition, Vector3d robotVelocity, Matrix3d robotRotation, double theta) {
      this.target = target;
      this.robotPosition = robotPosition;
      this.robotVelocity = robotVelocity;
      this.robotRotation = robotRotation;
      this.theta = theta;
    }

    public double[] value(double[] parameters) {
      double v = parameters[0];
      double alpha = parameters[1];
      double beta = parameters[2];

      Vector3d vBall = (new Vector3d(robotRotation.times(Matrix3d.fromYaw(alpha).times(Matrix3d.fromPitch(beta).times(new Vector3d(1, 0, 0)))).times(v)));

      double[] initial = {
        robotPosition.x(), robotPosition.y(), robotPosition.y(),
        robotVelocity.x()+vBall.x(), robotPosition.y()+vBall.y(), robotPosition.z()+vBall.z()
      };

      FirstOrderIntegrator integrator = new DormandPrince853Integrator(1.0e-8, 1.0, 1.0e-10, 1.0e-10);

      integrator.addEventHandler(new TargetEventHandler(target), 0.1, 1.0e-4, 1000);

      try {
        integrator.integrate(new BallPositionSolver(), 0.0, initial, 30.0, initial);
      } catch (Exception e) {
        return new double[] { 1e9, 1e9, 1e9, 1e9 };
      }

      // Math I blatantly stole form ai because it's currently 3:12 am

      double finalX = initial[0];
      double finalY = initial[1];
      double finalZ = initial[2];
      
      // Impact angle calculation
      double finalVz = initial[5];
      double finalVhoriz = Math.sqrt(initial[3]*initial[3] + initial[4]*initial[4]);
      double arrivalTheta = Math.atan2(finalVz, finalVhoriz);

      // Return residuals (distance from target and difference from desired angle)
      return new double[] { 
        finalX - target.x(), 
        finalY - target.y(), 
        finalZ - target.z(),
        arrivalTheta - theta 
      };
    }
  }

  public Vector3d solve(double[] initialGuess) {
    Optimizer model = new Optimizer(target, (Vector3d)robotPosition.getTranslation().toVector(), robotVelocity, (Matrix3d)robotPosition.getRotation().toMatrix(), theta);

    double[] targetError = {0, 0, 0, 0};

    double relTol = 1e-6;
    double absTol = 1e-6;
    EvaluationRmsChecker checker = new EvaluationRmsChecker(relTol, absTol);

    LeastSquaresProblem problem = LeastSquaresFactory.create(
      LeastSquaresFactory.model(model, params -> {
        int m = 4;
        int n = 3;
        double[][] jac = new double[m][n];
        double[] epsilon = {0.01, 0.001, 0.001};
        double[] base = model.value(params);
        for (int j = 0; j < n; j++) {
          double[] stepped = params.clone();
          stepped[j] += epsilon[j];
          double[] val = model.value(stepped);
          for (int i = 0; i < m; i++) jac[i][j] = (val[i] - base[i]) / epsilon[j];
        }
        return jac;
      }),
      new ArrayRealVector(targetError),
      new ArrayRealVector(initialGuess),
      checker,
      1000,
      1000
    );

    LevenbergMarquardtOptimizer optimizer = new LevenbergMarquardtOptimizer();
    LeastSquaresOptimizer.Optimum optimum = optimizer.optimize(problem);

    double[] solution = optimum.getPoint().toArray();

    return new Vector3d(solution[0], solution[1], solution[2]);
  }

}