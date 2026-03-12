package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CollectorConstants;

public class Collector extends SubsystemBase {
  public final SparkMax tiltMotor;
  public final SparkMax rollerMotor;

  public final RelativeEncoder tiltEncoder;
  public final RelativeEncoder rollerEncoder;
  
  public final DutyCycleEncoder tiltThruBoreEncoder;

  public final DigitalInput collectorBeamBreak;

  public Collector() {
    //instantiating motor controllers
    tiltMotor = new SparkMax(CollectorConstants.tiltMotorID, MotorType.kBrushless);
    rollerMotor = new SparkMax(CollectorConstants.rollerMotorID, MotorType.kBrushless);

    tiltEncoder = tiltMotor.getEncoder();
    rollerEncoder = rollerMotor.getEncoder();

    tiltThruBoreEncoder = new DutyCycleEncoder(CollectorConstants.tiltThruBoreEncoderID);

    //instantiating beam break
    collectorBeamBreak = new DigitalInput(CollectorConstants.collectorBeamBreakID);

    //tilt config
    SparkMaxConfig tiltConfig = new SparkMaxConfig();
    tiltConfig.inverted(CollectorConstants.tiltInverted);
    tiltConfig.idleMode(IdleMode.kBrake);
    tiltConfig.closedLoop.pid(CollectorConstants.kPTilt, CollectorConstants.kItilt, CollectorConstants.kDTilt);
    tiltConfig.closedLoop.positionWrappingEnabled(false);
    tiltConfig.encoder.positionConversionFactor(1.0/CollectorConstants.tiltGearRatio);
    tiltConfig.encoder.velocityConversionFactor(1.0/(60.0 * CollectorConstants.tiltGearRatio));
    tiltConfig.closedLoop.allowedClosedLoopError(0.01, ClosedLoopSlot.kSlot0);

    //soft limit config for tilt
    SoftLimitConfig tiltLimit = new SoftLimitConfig();
    tiltLimit.forwardSoftLimitEnabled(true);
    tiltLimit.reverseSoftLimitEnabled(true);
    tiltLimit.forwardSoftLimit(CollectorConstants.collectorInOffset);
    tiltLimit.reverseSoftLimit(CollectorConstants.collectorOutOffset);
    tiltConfig.apply(tiltLimit);

    tiltMotor.configure(tiltConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //roller config
    SparkMaxConfig rollerConfig = new SparkMaxConfig();
    rollerConfig.inverted(CollectorConstants.rollerInverted);
    rollerConfig.idleMode(IdleMode.kBrake);

    rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    //Thru Bore Encoder Config
    tiltThruBoreEncoder.setInverted(CollectorConstants.tiltThruBoreInverted);

    resetEncoders();
  }

  public void resetEncoders() {
    tiltEncoder.setPosition(tiltThruBoreEncoder.get() * (CollectorConstants.tiltThruBoreInverted ? -1.0 : 1.0));
    rollerEncoder.setPosition(0.0);
  }

  public boolean putCollectorIn() {
    SparkClosedLoopController tiltController = tiltMotor.getClosedLoopController();
    tiltController.setSetpoint(CollectorConstants.collectorInOffset, ControlType.kPosition);
    return tiltController.isAtSetpoint();
  }

  public boolean putCollectorOut() {
    SparkClosedLoopController tiltController = tiltMotor.getClosedLoopController();
    tiltController.setSetpoint(CollectorConstants.collectorOutOffset, ControlType.kPosition);
    return tiltController.isAtSetpoint();
  }

  public void setRollerSpeed(double speed) {
    rollerMotor.set(speed);
  }

  public void putThruBoreReading() {
    SmartDashboard.putNumber("ThruBoreReading", tiltThruBoreEncoder.get());
  }

  public void setTilt(double position) {
    if (position >= CollectorConstants.collectorInOffset && position <= CollectorConstants.collectorOutOffset) {
      SparkClosedLoopController tiltController = tiltMotor.getClosedLoopController();
      tiltController.setSetpoint(position, ControlType.kPosition);
    }
  }

  public boolean isAtSetpoint() {
    return tiltMotor.getClosedLoopController().isAtSetpoint();
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

  public boolean isNotePresent() {
    //returns false when beam is broken
    return collectorBeamBreak.get();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Tilt Motor Pos: ", tiltMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Thru Bore Pos: ", tiltThruBoreEncoder.get());
    if (collectorBeamBreak.get()) {
      SmartDashboard.putBoolean("Collector BB Intercepted", true);
    } else {
      SmartDashboard.putBoolean("Collector BB Intercepted", false);
    }
  }
}
