package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class SwerveModule extends SubsystemBase {
  private final SparkMax driveMotor;
  private final SparkMax rotationMotor;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder rotationEncoder;

  private final CANcoder absoluteEncoder;

  private final double absoluteEncoderOffset;
  private final boolean absoluteEncoderInverted;

  public SwerveModule(int driveMotorID, int rotationMotorID, int absoluteEncoderID, 
  double absoluteEncoderOffset, boolean absoluteEncoderInverted, boolean driveMotorInverted, boolean rotationMotorInverted) {
    //motors
    driveMotor = new SparkMax(driveMotorID, MotorType.kBrushless);
    rotationMotor = new SparkMax(rotationMotorID, MotorType.kBrushless);

    //encoders
    driveEncoder = driveMotor.getEncoder();
    rotationEncoder = rotationMotor.getEncoder();
    absoluteEncoder = new CANcoder(absoluteEncoderID);

    //drive motor config
    SparkMaxConfig driveConfig = new SparkMaxConfig();
    driveConfig.inverted(driveMotorInverted);
    driveConfig.idleMode(IdleMode.kBrake);
    driveConfig.closedLoop.pid(0.0, 0.0, 0.0);
    driveConfig.closedLoop.feedForward.apply(new FeedForwardConfig().sva(SwerveConstants.kSDrive, SwerveConstants.kVDrive, SwerveConstants.kADrive));
    driveConfig.encoder.positionConversionFactor((Math.PI * SwerveConstants.wheelDiameterMeters)/SwerveConstants.driveMotorGearRatio);
    driveConfig.encoder.velocityConversionFactor((Math.PI * SwerveConstants.wheelDiameterMeters)/(60*SwerveConstants.driveMotorGearRatio));
    driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //rotation motor config
    SparkMaxConfig rotationConfig = new SparkMaxConfig();
    rotationConfig.inverted(rotationMotorInverted);
    rotationConfig.idleMode(IdleMode.kBrake);
    rotationConfig.closedLoop.pid(SwerveConstants.kPRotation, SwerveConstants.kIRotation, SwerveConstants.kDRotation);
    rotationConfig.closedLoop.positionWrappingEnabled(true);
    rotationConfig.closedLoop.positionWrappingInputRange(-Math.PI, Math.PI);
    rotationConfig.encoder.positionConversionFactor(2*Math.PI/SwerveConstants.rotationMotorGearRatio);
    rotationConfig.encoder.velocityConversionFactor(2*Math.PI/(60 * SwerveConstants.rotationMotorGearRatio));
    rotationMotor.configure(rotationConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //CANcoder Config
    // MagnetSensorConfigs CANConfig = new MagnetSensorConfigs().withAbsoluteSensorDiscontinuityPoint(1.0);
    // absoluteEncoder.getConfigurator().apply(CANConfig);

    this.absoluteEncoderInverted = absoluteEncoderInverted;
    this.absoluteEncoderOffset = absoluteEncoderOffset;

    resetEncoders();
    straighten();
  }

  public double getDrivePosition() {
    return driveEncoder.getPosition();
  }

  public double getRotationPosition() {
    return rotationEncoder.getPosition();
  }

  public double getDriveVelocity() {
    return driveEncoder.getVelocity();
  }

  public double getRotationVelocity() {
    return rotationEncoder.getVelocity();
  }

  public double getAbsolutePosition() {
    return absoluteEncoder.getAbsolutePosition().getValueAsDouble();
  }

  public double getAbsolutePositionRad() {
    return getAbsolutePosition() * 2 * Math.PI * (absoluteEncoderInverted ? -1.0 : 1.0);
  }

  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getRotationPosition()));
  }

  public void resetEncoders() {
    //sets drive encoder to set the current position to 0
    //sets the rotation relative encoder sync with the absolute encoder
    driveEncoder.setPosition(0);

    //absoluteRadians (current reading) - offset radians (straight offset) = current angle (from straight)
    double absoluteRadians = getAbsolutePositionRad();

    rotationEncoder.setPosition(absoluteRadians - absoluteEncoderOffset);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getRotationPosition()));
  }

  public void straighten() {
    SparkClosedLoopController rotationController = rotationMotor.getClosedLoopController();
    rotationController.setSetpoint(0, ControlType.kPosition);
  }

  public void setDesiredState(SwerveModuleState state) {
    if (Math.abs(state.speedMetersPerSecond) < 0.001) {
      stop();
      return;
    }
    state.optimize(getState().angle);
    driveMotor.set(state.speedMetersPerSecond/SwerveConstants.physicalMaxSpeedMetersPerSec);
    SparkClosedLoopController rotationController = rotationMotor.getClosedLoopController();
    rotationController.setSetpoint(state.angle.getRadians(),ControlType.kPosition);
  }

  public void stop() {
    driveMotor.set(0);
    rotationMotor.set(0);
  }

  @Override
  public void periodic() {}
}
