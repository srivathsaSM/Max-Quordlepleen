package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class Constants {
    public static final int kJoystickPort = 0;
    public static final int kControllerPort = 1;

    public static final double rotToRad = 2 * Math.PI;

    public class CollectorConstants {
        //collector bindings
        public static final int collectButtonIndex = 12;

        //motor controller and encoder IDs
        public static final int tiltMotorID = 0;
        public static final int rollerMotorID = 0;
        public static final int tiltThruBoreEncoderID = 0;

        //inversions
        public static final boolean tiltInverted = false;
        public static final boolean rollerInverted = false;
        public static final boolean tiltThruBoreInverted = false;

        //PID
        public static final double kPTilt = 1.0;
        public static final double kItilt = 0.0;
        public static final double kDTilt = 0.0;

        //Thru Bore Encoder Offsets in rotations
        public static final double collectorInOffset = 0.0;
        public static final double collectorOutOffset = 0.0;
        
        //roller speed
        public static final double collectingRollerSpeed = 0.5;
    }

    public class SwerveConstants {
        //control bindings
        public static final int driverXAxis = 0;
        public static final int driverYAxis = 1;
        public static final int driverRotAxis = 2;
        public static final int sliderAxis = 3;
        public static final int driverFieldOrientedButtonIndex = 4;
        public static final int zeroHeadingButtonIndex = 2;
        public static final int straightenButtonIndex = 6;
        public static final int strafeOnlyButtonIndex = 9;
        public static final int invertedButtonIndex = 10;

        //swerve module controller and encoder IDs
        public static final int backRightDriveID = 1;
        public static final int backRightRotationID = 2;
        public static final int backRightEncoderID = 10;
        
        public static final int backLeftDriveID = 3;
        public static final int backLeftRotationID = 4;
        public static final int backLeftEncoderID = 20;

        public static final int frontRightDriveID = 5;
        public static final int frontRightRotationID = 6;
        public static final int frontRightEncoderID = 30;

        public static final int frontLeftDriveID = 7;
        public static final int frontLeftRotationID = 8;
        public static final int frontLeftEncoderID = 40;

        //rotation encoder offsets (RADIANS)
        public static final double backRightEncoderOffset = -0.426025 * Math.PI * 2;
        public static final double backLeftEncoderOffset = -0.652100 * Math.PI * 2;
        public static final double frontRightEncoderOffset = -0.456543  * Math.PI * 2;
        public static final double frontLeftEncoderOffset = -0.482422  * Math.PI * 2;

        //drive motor reversed states
        public static final boolean frontRightReversed = true;
        public static final boolean backRightReversed = true;
        public static final boolean frontLeftReversed = false;
        public static final boolean backLeftReversed = true;

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

        //Swerve RotationPID
        public static final double kPRotation = 0.2;
        public static final double kIRotation = 0;
        public static final double kDRotation = 0;

        //Drive Motor FeedForwards
        public static final double kSDrive = 0.2;
        public static final double kVDrive = 2.5;
        public static final double kADrive = 0.0;

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
