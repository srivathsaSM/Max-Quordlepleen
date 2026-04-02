package frc.robot.subsystems;


import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.NamedCommands;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.Shoot;

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
    shooterUpperConfig.closedLoop.pid(ShooterConstants.kPUpper, ShooterConstants.kIUpper, ShooterConstants.kDUpper);
    shooterUpperConfig.closedLoop.iZone(200);
    shooterUpperConfig.closedLoop.feedForward.kS(ShooterConstants.kSUpper).kV(ShooterConstants.kVUpper).kA(ShooterConstants.kAUpper);
    shooterUpperConfig.inverted(ShooterConstants.upperInverted);
    shooterUpperConfig.idleMode(IdleMode.kCoast);
    shooterUpperConfig.closedLoop.allowedClosedLoopError(50, ClosedLoopSlot.kSlot0);
    //shooterUpperConfig.encoder.quadratureMeasurementPeriod(100).quadratureAverageDepth(64); //measures positions every 20ms for velocity calculations (vs 100 default) and uses 16 positions to calculate velocity (vs default 64)
    shooterUpperMotor.configure(shooterUpperConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //shooter lower config
    SparkFlexConfig shooterLowerConfig = new SparkFlexConfig();
    shooterLowerConfig.closedLoop.pid(ShooterConstants.kPLower, ShooterConstants.kILower, ShooterConstants.kDLower);
    shooterLowerConfig.closedLoop.feedForward.apply(new FeedForwardConfig().sva(ShooterConstants.kSLower, ShooterConstants.kVLower, ShooterConstants.kALower));
    shooterLowerConfig.inverted(ShooterConstants.lowerInverted);
    shooterLowerConfig.idleMode(IdleMode.kCoast);
    shooterLowerConfig.closedLoop.allowedClosedLoopError(50, ClosedLoopSlot.kSlot0);
    shooterLowerMotor.configure(shooterLowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //indexer config
    SparkMaxConfig indexerConfig = new SparkMaxConfig();
    indexerConfig.idleMode(IdleMode.kBrake);
    indexerConfig.inverted(ShooterConstants.indexerInverted);
    indexerConfig.encoder.velocityConversionFactor(1.0/ShooterConstants.indexerGearRatio); //native velocity unit -> rpm
    indexerConfig.encoder.positionConversionFactor(1.0/ShooterConstants.indexerGearRatio);
    indexerMotor.configure(indexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    resetEncoders();

    upperRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(this::setVoltageUpper,
      log -> {
        log.motor("Upper Flywheel")
          .voltage(Volts.of(shooterUpperMotor.getAppliedOutput() * shooterUpperMotor.getBusVoltage()))
          .angularPosition(Angle.ofRelativeUnits(getUpperPosition(), Rotations))
          .angularVelocity(AngularVelocity.ofRelativeUnits(getUpperShooterVelocity(), RPM));
      },
      this)
    );

    lowerRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(this::setVoltageLower,
      log -> {
        log.motor("Lower Flywheel")
          .voltage(Volts.of(shooterLowerMotor.getAppliedOutput()*shooterLowerMotor.getBusVoltage()))
          .angularPosition(Angle.ofRelativeUnits(getLowerPosition(), Rotations))
          .angularVelocity(AngularVelocity.ofRelativeUnits(getLowerShooterVelocity(), RPM));
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

    upperController.setSetpoint(RPM * 1.035, ControlType.kVelocity);
    lowerController.setSetpoint(RPM * 1.035, ControlType.kVelocity);
  }

  public void runUpper(double RPM) {
    SparkClosedLoopController upperController = shooterUpperMotor.getClosedLoopController();
    upperController.setSetpoint(RPM * 1.035, ControlType.kVelocity);
  }

  public void runLower(double RPM) {
    SparkClosedLoopController lowerController = shooterLowerMotor.getClosedLoopController();
    lowerController.setSetpoint(RPM * 1.035, ControlType.kVelocity);
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

  public double getIndexerSpeed() {
    return indexerMotor.get();
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

  public void stopUpper() {
    shooterUpperMotor.set(0);
  }

  public void stopLower() {
    shooterLowerMotor.set(0);
  }

  public double upperFromSetpoint() {
    return shooterUpperMotor.getClosedLoopController().getSetpoint()/1.03 - shooterUpperEncoder.getVelocity();
  }

  public double lowerFromSetpoint() {
    return shooterLowerMotor.getClosedLoopController().getSetpoint()/1.03 - shooterLowerEncoder.getVelocity();
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

  public Command sysIdDynamicUpper(SysIdRoutine.Direction direction) {
    return upperRoutine.dynamic(direction);
  }

  public Command sysIdQuasistaticLower(SysIdRoutine.Direction direction) {
    return lowerRoutine.quasistatic(direction);
  }

  public Command sysIdDynamicLower(SysIdRoutine.Direction direction) {
    return lowerRoutine.dynamic(direction);
  }

  public void registerNamedCommands() {
    NamedCommands.registerCommand("FixedShoot", new Shoot(this));
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Load Beam Break: ", isNotePresent());

    SmartDashboard.putNumber("Upper Error", upperFromSetpoint());
    SmartDashboard.putNumber("Lower Error", lowerFromSetpoint());
    
    SmartDashboard.putNumber("Indexer Spd", getIndexerSpeed());
    SmartDashboard.putNumber("Upper RPM", getUpperShooterVelocity());
    SmartDashboard.putNumber("Lower RPM", getLowerShooterVelocity());

    Logger.recordOutput("Shooter/UpperRPM", getUpperShooterVelocity(), "RPM");
    Logger.recordOutput("Shooter/LowerRPM", getLowerShooterVelocity(), "RPM");
  }
}
