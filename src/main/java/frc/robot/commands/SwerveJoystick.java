package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.*;
import frc.robot.subsystems.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SwerveJoystick extends Command {
  private final SwerveSubsystem swerveSubsystem;
  private final Supplier<Double> xSpdFunction, ySpdFunction, rotSpdFunction;
  private final Supplier<Boolean> fieldOrientedFunction;
  private final SlewRateLimiter xLimiter, yLimiter, rotLimiter;
  

  public SwerveJoystick(SwerveSubsystem swerveSubsystem, Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> rotSpdFunction, Supplier<Boolean> fieldOrientedFunction) {
    this.swerveSubsystem = swerveSubsystem;
    this.xSpdFunction = xSpdFunction;
    this.ySpdFunction = ySpdFunction;
    this.rotSpdFunction = rotSpdFunction;
    this.fieldOrientedFunction = fieldOrientedFunction;
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
    double xSpeed = xSpdFunction.get();
    double ySpeed = ySpdFunction.get();
    double rotSpeed = rotSpdFunction.get();

    //deadband
    xSpeed = Math.abs(xSpeed) > SwerveConstants.deadband ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > SwerveConstants.deadband ? ySpeed : 0.0;
    rotSpeed = Math.abs(rotSpeed) > SwerveConstants.rotDeadband ? rotSpeed : 0.0;

    //squaring
    xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
    ySpeed = Math.copySign(ySpeed * ySpeed, ySpeed);
    rotSpeed = Math.copySign(rotSpeed * rotSpeed, rotSpeed);

    //limiters -> smoother driving
    xSpeed = xLimiter.calculate(xSpeed) * SwerveConstants.physicalMaxSpeedMetersPerSec;
    ySpeed = yLimiter.calculate(ySpeed) * SwerveConstants.physicalMaxSpeedMetersPerSec;
    rotSpeed = rotLimiter.calculate(rotSpeed) * SwerveConstants.maxAngularSpeedRadPerSec;

    //consider deleting this for drift problems (according to GPT)
    // if (xSpeed == 0 && ySpeed == 0 && rotSpeed == 0) {
    //   swerveSubsystem.stopModules();
    //   return;
    // }

    //constructs chassisSpeeds
    ChassisSpeeds chassisSpeeds;
    if (fieldOrientedFunction.get()) {
      //relative to field 
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, swerveSubsystem.getRotation2d());
    } else {
      //relative to robot
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rotSpeed);
    }

    //convert chassis speeds to individual module states
    SwerveModuleState[] moduleStates = SwerveConstants.driveKinematics.toSwerveModuleStates(chassisSpeeds);

    //Output module states to wheels
    swerveSubsystem.setModuleStates(moduleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
