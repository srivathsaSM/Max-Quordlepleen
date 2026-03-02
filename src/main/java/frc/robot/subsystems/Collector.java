package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CollectorConstants;

public class Collector extends SubsystemBase {
  public final SparkMax tiltMotor;
  public final SparkMax rollerMotor;

  public final RelativeEncoder tiltEncoder;
  public final RelativeEncoder rollerEncoder;
  
  public final DutyCycleEncoder tiltThruBoreEncoder;

  public Collector() {
    //instantiating motor controllers
    tiltMotor = new SparkMax(CollectorConstants.tiltMotorID, MotorType.kBrushless);
    rollerMotor = new SparkMax(CollectorConstants.rollerMotorID, MotorType.kBrushless);

    tiltEncoder = tiltMotor.getEncoder();
    rollerEncoder = rollerMotor.getEncoder();

    tiltThruBoreEncoder = new DutyCycleEncoder(CollectorConstants.tiltThruBoreEncoderID);

    //tilt config
    SparkMaxConfig tiltConfig = new SparkMaxConfig();
    tiltConfig.inverted(CollectorConstants.tiltInverted);
    tiltConfig.idleMode(IdleMode.kBrake);
    tiltConfig.closedLoop.pid(CollectorConstants.kPTilt, CollectorConstants.kItilt, CollectorConstants.kDTilt);
    tiltConfig.closedLoop.positionWrappingEnabled(false);

    //soft limit config for tilt
    SoftLimitConfig tiltLimit = new SoftLimitConfig();
    tiltLimit.forwardSoftLimitEnabled(true);
    tiltLimit.reverseSoftLimitEnabled(true);
    tiltLimit.forwardSoftLimit(CollectorConstants.collectorOutOffset);
    tiltLimit.reverseSoftLimit(CollectorConstants.collectorInOffset);
    tiltConfig.apply(tiltLimit);

    tiltMotor.configure(tiltConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //roller config
    SparkMaxConfig rollerConfig = new SparkMaxConfig();
    rollerConfig.inverted(CollectorConstants.rollerInverted);
    rollerConfig.idleMode(IdleMode.kBrake);

    rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    //Thru Bore Encoder Config
    tiltThruBoreEncoder.setInverted(CollectorConstants.tiltThruBoreInverted);
  }

  public void resetEncoders() {
    tiltEncoder.setPosition(tiltThruBoreEncoder.get() * (CollectorConstants.tiltThruBoreInverted ? -1.0 : 1.0));
    rollerEncoder.setPosition(0.0);
  }

  public void putCollectorIn() {
    SparkClosedLoopController tiltController = tiltMotor.getClosedLoopController();
    tiltController.setSetpoint(CollectorConstants.collectorInOffset, ControlType.kPosition);
  }

  public void putCollectorOut() {
    SparkClosedLoopController tiltController = tiltMotor.getClosedLoopController();
    tiltController.setSetpoint(CollectorConstants.collectorOutOffset, ControlType.kPosition);
  }

  public void setRollerSpeed(double speed) {
    rollerMotor.set(speed);
  }

  public void setTilt(double position) {
    if (position >= CollectorConstants.collectorInOffset && position <= CollectorConstants.collectorOutOffset) {
      SparkClosedLoopController tiltController = tiltMotor.getClosedLoopController();
      tiltController.setSetpoint(position, ControlType.kPosition);
    }
  }

  public void stopEverything() {
    tiltMotor.set(0);
    rollerMotor.set(0);
  }

  public void stopRoller() {
    rollerMotor.set(0);
  }

  public void stopTilt() {
    tiltMotor.set(0);
  }

  @Override
  public void periodic() {}
}
