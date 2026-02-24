package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class Constants {
    public static final int kDriverControllerPort = 0;

    public static final double rotToRad = 2 * Math.PI;

    public class CollectorConstants {
        //control bindings
        public static final int collectorInButtonIndex = 0;
        public static final int collectorOutButtonIndex = 0;
        public static final int tiltToPosButtonIndex = 0;
        public static final int tiltToPosAxisIndex = 4;

        //motor and encoder IDS
        public static final int tiltMotorID = 0;
        public static final int rollerMotorID = 0;
        public static final int tiltTBEncoderID = 0;

        //inversions
        public static final boolean tiltInverted = false;
        public static final boolean rollerInverted = false;
        public static final boolean tiltTBEncoderInverted = false;

        //encoder offsets
        public static double collectorIn = 0;
        public static double collectorOut = 0;

        //PID
        public static double cPTilt = 1;
        public static double cITilt = 0;
        public static double cDTilt = 0;
    }

    public class SwerveConstants {
        //control bindings
        public static final int driverXAxis = 0;
        public static final int driverYAxis = 1;
        public static final int driverRotAxis = 2;
        public static final int driverFieldOrientedButtonIndex = 1;
        public static final int zeroHeadingButtonIndex = 2;

        //swerve module controller and encoder IDs
        public static final int backRightDriveID = 3;
        public static final int backRightRotationID = 4;
        public static final int backRightEncoderID = 40;
        
        public static final int backLeftDriveID = 1;
        public static final int backLeftRotationID = 2;
        public static final int backLeftEncoderID = 50;

        public static final int frontRightDriveID = 7;
        public static final int frontRightRotationID = 8;
        public static final int frontRightEncoderID = 30;

        public static final int frontLeftDriveID = 5;
        public static final int frontLeftRotationID = 6;
        public static final int frontLeftEncoderID = 20;

        //rotation encoder offsets (RADIANS)
        public static final double backRightEncoderOffset = Units.rotationsToRadians(0.661);
        public static final double backLeftEncoderOffset = Units.rotationsToRadians(0.197);
        public static final double frontRightEncoderOffset = Units.rotationsToRadians(0.143);
        public static final double frontLeftEncoderOffset = Units.rotationsToRadians(0.992);

        //drive motor reversed states
        public static final boolean frontRightReversed = false;
        public static final boolean backRightReversed = false;
        public static final boolean frontLeftReversed = false;
        public static final boolean backLeftReversed = false;

        //rotation encoder reversed 
        public static final boolean frontRightAbsReversed = true;
        public static final boolean backRightAbsReversed = true;
        public static final boolean frontLeftAbsReversed = true;
        public static final boolean backLeftAbsReversed = true;

        //rotation Motor reversed
        public static final boolean frontRightRotReversed = false;
        public static final boolean backRightRotReversed = false;
        public static final boolean frontLeftRotReversed = false;
        public static final boolean backLeftRotReversed = false;

        //Swerve PID
        public static final double kPRotation = 0.3;
        public static final double kIRotation = 0;
        public static final double kDRotation = 0;

        //swerve module hardware specifications
        public static final double wheelDiameterMeters = Units.inchesToMeters(4.0/1.0);
        public static final double driveMotorGearRatio = (6.75 / 1.0); // 6.75:1
        public static final double rotationMotorGearRatio = ((150.0 / 7.0) / 1.0); // 150/7:1
        public static final double driveEncoderRotationToMeters = driveMotorGearRatio * rotToRad * wheelDiameterMeters/2;
        public static final double rotationEncoderRotationToRad = rotationMotorGearRatio * rotToRad;
        public static final double driveEncoderRPMToMetersPerSec = driveEncoderRotationToMeters/60;
        public static final double rotationEncoderRPMToRadPerSec = rotationEncoderRotationToRad/60;

        //constraints
        public static final double physicalMaxSpeedMetersPerSec = 7.5/4.0;
        public static final double maxAngularSpeedRadPerSec = 3.5;
        public static final double driveMaxAccel = 3.0;
        public static final double angularMaxAccel = 4 * Math.PI;

        //deadband
        public static final double deadband = 0.15;
        public static final double rotDeadband = 0.6;

        //distance between right and left wheels
        public static final double trackWidth = Units.inchesToMeters(22.66);
        
        //distance between front and back wheels
        public static final double wheelBase = Units.inchesToMeters(24.5);

        public static final SwerveDriveKinematics driveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase/2, -trackWidth/2), //front left
        new Translation2d(wheelBase/2, trackWidth/2),  //front right
        new Translation2d(-wheelBase/2, -trackWidth/2), //back left
        new Translation2d(-wheelBase/2, trackWidth/2));  //back right    
    }
}
