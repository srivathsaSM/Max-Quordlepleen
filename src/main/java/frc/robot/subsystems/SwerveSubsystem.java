package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class SwerveSubsystem extends SubsystemBase {
  private final SwerveModule frontLeft = new SwerveModule(
          SwerveConstants.frontLeftDriveID, 
          SwerveConstants.frontLeftRotationID, 
          SwerveConstants.frontLeftEncoderID,
          SwerveConstants.frontLeftEncoderOffset,
          SwerveConstants.frontLeftAbsReversed,
          SwerveConstants.frontLeftReversed,
          SwerveConstants.frontLeftRotReversed);

  private final SwerveModule frontRight = new SwerveModule(
          SwerveConstants.frontRightDriveID, 
          SwerveConstants.frontRightRotationID, 
          SwerveConstants.frontRightEncoderID,
          SwerveConstants.frontRightEncoderOffset,
          SwerveConstants.frontRightAbsReversed,
          SwerveConstants.frontRightReversed,
          SwerveConstants.frontRightRotReversed);

  private final SwerveModule backLeft = new SwerveModule(
          SwerveConstants.backLeftDriveID, 
          SwerveConstants.backLeftRotationID, 
          SwerveConstants.backLeftEncoderID,
          SwerveConstants.backLeftEncoderOffset,
          SwerveConstants.backLeftAbsReversed,
          SwerveConstants.backLeftReversed,
          SwerveConstants.backLeftRotReversed);
  
  private final SwerveModule backRight = new SwerveModule(
          SwerveConstants.backRightDriveID, 
          SwerveConstants.backRightRotationID, 
          SwerveConstants.backRightEncoderID,
          SwerveConstants.backRightEncoderOffset,
          SwerveConstants.backRightAbsReversed,
          SwerveConstants.backRightReversed,
          SwerveConstants.backRightRotReversed);

  private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

  public final SwerveModulePosition[] zeroModulePositions = {
    new SwerveModulePosition(0,new Rotation2d(0)),
    new SwerveModulePosition(0,new Rotation2d(0)),
    new SwerveModulePosition(0,new Rotation2d(0)),
    new SwerveModulePosition(0,new Rotation2d(0))
  };

  private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(SwerveConstants.driveKinematics, new Rotation2d(0), zeroModulePositions);
  
  public SwerveSubsystem() {
      new Thread(() -> {
      try {
        Thread.sleep(1000);
        zeroHeading();
      } catch (Exception e) {
      }
    }).start();
  }

  public void zeroHeading() {
    gyro.reset();
  }

  public double getHeading() {
    return Math.IEEEremainder(gyro.getAngle(), 360);
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  public Pose2d getPose() {
    return odometer.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    SwerveModulePosition[] modulePositions = {
      frontLeft.getModulePosition(),
      frontRight.getModulePosition(),
      backLeft.getModulePosition(),
      backRight.getModulePosition()};

    odometer.resetPosition(getRotation2d(), modulePositions, pose);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    SwerveModuleState[] states = {
      frontLeft.getState(),
      frontRight.getState(),
      backLeft.getState(),
      backRight.getState()};

      return SwerveConstants.driveKinematics.toChassisSpeeds(states);
  }

  public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
    ChassisSpeeds.discretize(chassisSpeeds, 0.02);
    setModuleStates(SwerveConstants.driveKinematics.toSwerveModuleStates(chassisSpeeds));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Robot Heading", getHeading());

    SmartDashboard.putNumber("Rot FR", frontRight.getAbsolutePositionRad());
    SmartDashboard.putNumber("Rot FL", frontLeft.getAbsolutePositionRad());
    SmartDashboard.putNumber("Rot BR", backRight.getAbsolutePositionRad());
    SmartDashboard.putNumber("Rot BL", backLeft.getAbsolutePositionRad());

    SmartDashboard.putNumber("State Radians FR: ", frontRight.getState().angle.getRadians());
    SmartDashboard.putNumber("Actual Radians FR: ", frontRight.getRotationPosition());

    SmartDashboard.putNumber("State Radians FL: ", frontLeft.getState().angle.getRadians());
    SmartDashboard.putNumber("Actual Radians FL: ", frontLeft.getRotationPosition());

    SmartDashboard.putNumber("State Radians BR: ", backRight.getState().angle.getRadians());
    SmartDashboard.putNumber("Actual Radians BR: ", backRight.getRotationPosition());

    SmartDashboard.putNumber("State Radians BL: ", backLeft.getState().angle.getRadians());
    SmartDashboard.putNumber("Actual Radians BL: ", backLeft.getRotationPosition());

    SwerveModulePosition[] modulePositions = {
      frontLeft.getModulePosition(),
      frontRight.getModulePosition(),
      backLeft.getModulePosition(),
      backRight.getModulePosition()
    };

    odometer.update(getRotation2d(), modulePositions);

    SmartDashboard.putNumber("Robot Heading: ", getHeading());
    SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
  }

  public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }

  public SwerveModule[] getModules() {
    SwerveModule[] modules = {frontLeft, frontRight, backLeft, backRight};
    return modules;
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.physicalMaxSpeedMetersPerSec);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }

  public void configureAutoBuilder() {
    System.out.println("Configuring Auto Builder... ");

    try {
      RobotConfig config = RobotConfig.fromGUISettings();

      //configure autobuilder
      AutoBuilder.configure(
        this::getPose, //robot pose supplier
        this::resetOdometry, //method to reset odometry
        this::getRobotRelativeSpeeds, //robot relative ChassisSpeeds supplier
        (speeds, feedforwards) -> driveRobotRelative(speeds), //drive the robot with chassis speeds and feedforwards
        new PPHolonomicDriveController(
          new PIDConstants(5.0, 0.0, 0.0), //translation PID
          new PIDConstants(5.0, 0.0, 0.0)), //rotation PID
        config, //robotconfig
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this);
    } catch (Exception e) {
      e.printStackTrace();
    }
  }
}
