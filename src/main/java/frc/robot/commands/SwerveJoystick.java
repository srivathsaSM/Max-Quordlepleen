package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.*;
import frc.robot.subsystems.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SwerveJoystick extends Command {
  private final SwerveSubsystem swerveSubsystem;
  private final DoubleSupplier xSpdFunction, ySpdFunction, rotSpdFunction, sliderFunction;
  private final BooleanSupplier strafeOnly, inverted;
  private final SlewRateLimiter xLimiter, yLimiter, rotLimiter;
  

  public SwerveJoystick(SwerveSubsystem swerveSubsystem, DoubleSupplier xSpdFunction, DoubleSupplier ySpdFunction, 
  DoubleSupplier rotSpdFunction, DoubleSupplier sliderFunction, BooleanSupplier strafeOnly, BooleanSupplier inverted) {
    this.swerveSubsystem = swerveSubsystem;
    this.xSpdFunction = xSpdFunction;
    this.ySpdFunction = ySpdFunction;
    this.rotSpdFunction = rotSpdFunction;
    this.sliderFunction = sliderFunction;
    this.strafeOnly = strafeOnly;
    this.inverted = inverted;
    this.xLimiter = new SlewRateLimiter(SwerveConstants.driveMaxAccel);
    this.yLimiter = new SlewRateLimiter(SwerveConstants.driveMaxAccel);
    this.rotLimiter = new SlewRateLimiter(SwerveConstants.angularMaxAccel);
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //get real time joystick inputs
    double xSpeed = xSpdFunction.getAsDouble();
    double ySpeed = ySpdFunction.getAsDouble();
    double rotSpeed = rotSpdFunction.getAsDouble();

    double sliderValue = (-sliderFunction.getAsDouble() + 1.0)/2.0;
    sliderValue = sliderValue < 0.15 ? 0.15 : sliderValue;
    SmartDashboard.putNumber("Slider", sliderValue);

    //deadband
    xSpeed = Math.abs(xSpeed) > 0.25 ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > 0.35 ? ySpeed : 0.0;
    rotSpeed = Math.abs(rotSpeed) > 0.4 ? rotSpeed : 0.0;

    xSpeed = MathUtil.applyDeadband(xSpeed, 0.1, 1);
		ySpeed = MathUtil.applyDeadband(ySpeed, 0.1, 1);
		rotSpeed = MathUtil.applyDeadband(rotSpeed, 0.3, 0.75);

    //limiters -> smoother driving
    xSpeed = xLimiter.calculate(xSpeed) * SwerveConstants.physicalMaxSpeedMetersPerSec * sliderValue;
    ySpeed = yLimiter.calculate(ySpeed) * SwerveConstants.physicalMaxSpeedMetersPerSec * sliderValue;
    rotSpeed = rotLimiter.calculate(rotSpeed) * SwerveConstants.maxAngularSpeedRadPerSec * sliderValue;

    if (inverted.getAsBoolean()) {
      swerveSubsystem.drive(-xSpeed, ySpeed, -rotSpeed, swerveSubsystem.isFieldOriented);
    } else if (strafeOnly.getAsBoolean()) {
      swerveSubsystem.drive(0, ySpeed, 0, true);
    } else {
      swerveSubsystem.drive(xSpeed, ySpeed, rotSpeed, swerveSubsystem.isFieldOriented);
    }

    //consider deleting this for drift problems (according to GPT)
    // if (xSpeed == 0 && ySpeed == 0 && rotSpeed == 0) {
    //   swerveSubsystem.stopModules();
    //   return;
    // }

    // //constructs chassisSpeeds
    // ChassisSpeeds chassisSpeeds;
    // if (fieldOrientedFunction.getAsBoolean()) {
    //   //relative to field 
    //   chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, swerveSubsystem.getRotation2d());
    // } else {
    //   //relative to robot
    //   chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rotSpeed);
    // }

    // //convert chassis speeds to individual module states
    // SwerveModuleState[] moduleStates = SwerveConstants.driveKinematics.toSwerveModuleStates(chassisSpeeds);

    // //Output module states to wheels
    // swerveSubsystem.setModuleStates(moduleStates);
  }

  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.drive(0.0, 0.0, 0.0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
