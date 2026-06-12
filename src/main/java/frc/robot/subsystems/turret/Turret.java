package frc.robot.subsystems.turret;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.utils.ContinuousAbsoluteEncoder;

import static com.revrobotics.PersistMode.kPersistParameters;
import static com.revrobotics.ResetMode.kResetSafeParameters;
import static com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless;
import static com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kCoast;
import static frc.robot.subsystems.turret.TurretConstants.*;

public class Turret extends SubsystemBase {
  private final Servo linearActuator = new Servo(0);
  private final Servo linearActuator2 = new Servo(1);

  private double linearActuatorPosition = 0;

  private double panSetpoint = 0;

  private final SparkMax panMotor;
  private final AbsoluteEncoder panEncoderAbsolute;

  private final double panEncoderPositionFactor = 2.0 * Math.PI * (28.0 / 200.0);

  private final ContinuousAbsoluteEncoder panEncoder;

  private final PIDController panPidController = new PIDController(1, 0, 0);

  private static final Turret instance = new Turret();
  public static Turret getInstance() {
    return instance;
  }

  private Turret() {
    linearActuator.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
    linearActuator2.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);

    SparkMaxConfig panConfig = new SparkMaxConfig();
    panConfig.smartCurrentLimit(10).idleMode(kCoast).inverted(true);
    panConfig.voltageCompensation(12.0);
    panConfig.absoluteEncoder.positionConversionFactor(panEncoderPositionFactor);
    panConfig.absoluteEncoder.inverted(false);
    panConfig.openLoopRampRate(0.2);
    panMotor = new SparkMax(31, kBrushless);
    panMotor.configure(panConfig, kResetSafeParameters, kPersistParameters);

    panEncoderAbsolute = panMotor.getAbsoluteEncoder();

    panEncoder = new ContinuousAbsoluteEncoder();
    panEncoder.setEncoderConversionFactor(panEncoderPositionFactor);
  }

  public void periodic() {
    panEncoder.update(panEncoderAbsolute.getPosition());
    if (RobotContainer.getInstance().isBeforeFirstEnable()) {
      if (panEncoderAbsolute.getPosition() > .5 * panEncoderPositionFactor) {
        panEncoder.setAccumulator(-1);
      } else {
        panEncoder.setAccumulator(0);
      }
    }

    SmartDashboard.putNumber("panEncoder", panEncoder.getPosition());
    SmartDashboard.putNumber("panSetpoint", panSetpoint);
  }

  public void setLinearActuator(double value) {
    double clamped =
        MathUtil.clamp(value, 0, 100);
    linearActuatorPosition = clamped;
    double newSetpoint = (clamped / 100) * 2 - 1;
    linearActuator.setSpeed(newSetpoint);
    linearActuator2.setSpeed(newSetpoint);
  }

  public double alterLinearActuator(double value) {
    setLinearActuator(linearActuatorPosition + value);
    return linearActuatorPosition;
  }

  public void setPanSetpoint(double setpoint) {
    panSetpoint = MathUtil.clamp(MathUtil.inputModulus(setpoint, yawModuloMin, yawModuloMax), yawMin, yawMax);
    panMotor.set(MathUtil.clamp(panPidController.calculate(panEncoder.getPosition(), panSetpoint), -maxYawOutput, maxYawOutput));
  }

  public boolean isPanAtSetpoint() {
    // if the turret is not in automatic mode (ie either it is not controlled at all
    // or controlled manually by the driver) then it is at the setpoint, in a way.
    if (!(getCurrentCommand() instanceof AutoTurret)) return true;
   return Math.abs(panEncoder.getPosition() - panSetpoint) < 0.1;
  }

  public void setPanMotor(double output) {
    panMotor.set(output);
  }
}
