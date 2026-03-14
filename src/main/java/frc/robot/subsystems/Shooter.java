package frc.robot.subsystems;


import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {

  private final SparkFlex shooterUpperMotor;
  private final SparkFlex shooterLowerMotor;

  private final SparkMax indexerMotor;

  private final RelativeEncoder shooterUpperEncoder;
  private final RelativeEncoder shooterLowerEncoder;

  private final DigitalInput indexBeamBreak;

  private final SysIdRoutine upperRoutine;
  private final SysIdRoutine lowerRoutine;

  public Shooter() {
    shooterUpperMotor = new SparkFlex(ShooterConstants.shooterUpperID, MotorType.kBrushless);
    shooterLowerMotor = new SparkFlex(ShooterConstants.shooterLowerID, MotorType.kBrushless);
    
    indexerMotor = new SparkMax(ShooterConstants.indexerID, MotorType.kBrushless);

    shooterUpperEncoder = shooterUpperMotor.getEncoder();
    shooterLowerEncoder = shooterLowerMotor.getEncoder();

    indexBeamBreak = new DigitalInput(ShooterConstants.indexBeamBreakID);

    //shooter upper config
    SparkFlexConfig shooterUpperConfig = new SparkFlexConfig();
    shooterUpperConfig.closedLoop.pid(ShooterConstants.kPShoot, ShooterConstants.kIShoot, ShooterConstants.kDShoot);
    shooterUpperConfig.inverted(ShooterConstants.upperInverted);
    shooterUpperConfig.idleMode(IdleMode.kCoast);
    shooterUpperConfig.closedLoop.allowedClosedLoopError(50, ClosedLoopSlot.kSlot0);
    shooterUpperMotor.configure(shooterUpperConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //shooter lower config
    SparkFlexConfig shooterLowerConfig = new SparkFlexConfig();
    shooterLowerConfig.apply(shooterUpperConfig);
    shooterLowerConfig.inverted(ShooterConstants.lowerInverted);
    shooterLowerConfig.idleMode(IdleMode.kCoast);
    shooterLowerConfig.closedLoop.allowedClosedLoopError(50, ClosedLoopSlot.kSlot0);
    shooterLowerMotor.configure(shooterLowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //indexer config
    SparkMaxConfig indexerConfig = new SparkMaxConfig();
    indexerConfig.idleMode(IdleMode.kBrake);
    indexerConfig.inverted(ShooterConstants.indexerInverted);
    indexerConfig.encoder.velocityConversionFactor(1.0/ShooterConstants.indexerGearRatio);
    indexerConfig.encoder.positionConversionFactor(1.0/ShooterConstants.indexerGearRatio);
    indexerMotor.configure(indexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    resetEncoders();

    upperRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(this::setVoltageUpper,
      log -> {
        log.motor("Upper Flywheel")
          .voltage(Volts.of(shooterUpperMotor.getAppliedOutput() * shooterUpperMotor.getBusVoltage()))
          .angularPosition(Angle.ofBaseUnits(getUpperPosition(), Rotations))
          .angularVelocity(AngularVelocity.ofBaseUnits(getUpperShooterVelocity(), RPM));
      },
      this)
    );

    lowerRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(this::setVoltageLower,
      log -> {
        log.motor("Lower Flywheel")
          .voltage(Volts.of(shooterLowerMotor.getAppliedOutput()*shooterLowerMotor.getBusVoltage()))
          .angularPosition(Angle.ofBaseUnits(getLowerPosition(), Rotations))
          .angularVelocity(AngularVelocity.ofBaseUnits(getLowerShooterVelocity(), RPM));
      },
      this)
    );
  }

  public void resetEncoders() {
    shooterUpperEncoder.setPosition(0.0);
    shooterLowerEncoder.setPosition(0.0);
  }

  public void runShooter(double RPM) {
    SparkClosedLoopController upperController = shooterUpperMotor.getClosedLoopController();
    SparkClosedLoopController lowerController = shooterLowerMotor.getClosedLoopController();

    upperController.setSetpoint(RPM, ControlType.kVelocity);
    lowerController.setSetpoint(RPM, ControlType.kVelocity);
  }

  public void runIndexer(double speed) {
    indexerMotor.set(speed);
  }

  public double getUpperPosition() {
    return shooterUpperEncoder.getPosition();
  }

  public double getLowerPosition() {
    return shooterLowerEncoder.getPosition();
  }

  public double getUpperShooterVelocity() {
    return shooterUpperEncoder.getVelocity();
  }

  public double getLowerShooterVelocity() {
    return shooterLowerEncoder.getVelocity();
  }

  public void stopAll() {
    indexerMotor.set(0.0);
    shooterLowerMotor.set(0.0);
    shooterUpperMotor.set(0.0);
  }

  public void stopIndexer() {
    indexerMotor.set(0);
  }

  public boolean isNotePresent() {
    return !indexBeamBreak.get();
  }

  public void stopShooter() {
    shooterLowerMotor.set(0);
    shooterUpperMotor.set(0);
  }

  public void setVoltageUpper(Voltage voltage) {
    shooterUpperMotor.setVoltage(voltage);
  }

  public void setVoltageLower(Voltage voltage) {
    shooterLowerMotor.setVoltage(voltage);
  }

  public void setVoltageBoth(Voltage voltage) {
    setVoltageUpper(voltage);
    setVoltageLower(voltage);
  }

  public Command sysIdQuasistaticUpper(SysIdRoutine.Direction direction) {
    return upperRoutine.quasistatic(direction);
  }

  public Command sysIdDyanamicUpper(SysIdRoutine.Direction direction) {
    return upperRoutine.dynamic(direction);
  }

  public Command sysIdQuasistaticLower(SysIdRoutine.Direction direction) {
    return lowerRoutine.quasistatic(direction);
  }

  public Command sysIdDynamicLower(SysIdRoutine.Direction direction) {
    return lowerRoutine.dynamic(direction);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Load Beam Break: ", isNotePresent());
  }
}
