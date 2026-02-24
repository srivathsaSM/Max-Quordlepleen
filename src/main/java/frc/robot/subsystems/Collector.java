package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class Collector extends SubsystemBase {
  private final SparkMax rollerMotor;
  private final SparkMax tiltMotor;

  private final RelativeEncoder rollerEncoder;
  private final RelativeEncoder tiltEncoder;

  private final DutyCycleEncoder tiltTBEncoder;

  public Collector() {
    //instantiating motors + encoder
    tiltMotor = new SparkMax(CollectorConstants.tiltMotorID, MotorType.kBrushless);
    rollerMotor = new SparkMax(CollectorConstants.rollerMotorID, MotorType.kBrushless);

    tiltEncoder = tiltMotor.getEncoder();
    rollerEncoder = rollerMotor.getEncoder();

    tiltTBEncoder = new DutyCycleEncoder(CollectorConstants.tiltTBEncoderID);

    //tilt config
    SparkMaxConfig tiltConfig = new SparkMaxConfig();
    tiltConfig.inverted(CollectorConstants.tiltInverted);
    tiltConfig.idleMode(IdleMode.kBrake);
    tiltConfig.closedLoop.pid(CollectorConstants.cPTilt, CollectorConstants.cITilt, CollectorConstants.cDTilt);
    tiltConfig.closedLoop.positionWrappingEnabled(false);

    // SoftLimitConfig tiltSoftLimit = new SoftLimitConfig();
    // tiltSoftLimit.forwardSoftLimitEnabled(true);
    // tiltSoftLimit.reverseSoftLimitEnabled(true);
    // tiltSoftLimit.forwardSoftLimit(CollectorConstants.collectorOut);
    // tiltSoftLimit.reverseSoftLimit(CollectorConstants.collectorIn);

    //tiltConfig.apply(tiltSoftLimit);
    tiltMotor.configure(tiltConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //roller config
    SparkMaxConfig rollerConfig = new SparkMaxConfig();
    rollerConfig.inverted(CollectorConstants.rollerInverted);
    rollerConfig.idleMode(IdleMode.kCoast);
    rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //Through Bore Encoder Config
    tiltTBEncoder.setInverted(CollectorConstants.tiltTBEncoderInverted);
    tiltTBEncoder.setDutyCycleRange(CollectorConstants.collectorIn, CollectorConstants.collectorOut);

    resetEncoders();
  }

  public void resetEncoders() {
    tiltEncoder.setPosition(tiltTBEncoder.get() * (CollectorConstants.tiltTBEncoderInverted ? 1.0 : -1.0));
    rollerEncoder.setPosition(0);
  }

  public void putIn() {
    SparkClosedLoopController tiltController = tiltMotor.getClosedLoopController();
    tiltController.setSetpoint(CollectorConstants.collectorIn, ControlType.kPosition);
  }

  public void putOut() {
    SparkClosedLoopController tiltController = tiltMotor.getClosedLoopController();
    tiltController.setSetpoint(CollectorConstants.collectorOut, ControlType.kPosition);
  }

  public void setRollerSpeed(double speed) {
    rollerMotor.set(speed);
  }

  public void setTilt(double position) {
    if (position >= 0 && position <= 1) {
      SparkClosedLoopController tiltController = tiltMotor.getClosedLoopController();
      tiltController.setSetpoint(position, ControlType.kPosition);
    }
  }

  public void stop() {
    tiltMotor.set(0);
    rollerMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
