// FRC2106 Junkyard Dogs - Continuity Base Code - www.team2106.org

package frc.robot.util;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {

    // Swerve modules
    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1 / 6.75 ; // old value 1 / 5.8462
        public static final double kTurningMotorGearRatio = 1 / 21.4285714286; // old value 1 / 18.0
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;

        // Used in working code currently
        public static final double kPTurning = 0.5;

        // These two used for simulation currently 
        public static final double kITurning = 0.0;
        public static final double kDTurning = 0.005;
        
    }

    // Swerve drive
    public static final class DriveConstants {

        // Distance between right and left wheels
        public static final double kTrackWidth = Units.inchesToMeters(26.75);

        // Distance between front and back wheels
        public static final double kWheelBase = Units.inchesToMeters(22.75);

        // Need to update to correct values, I dont remember the value we set last meet
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2), //fl
                new Translation2d(kWheelBase / 2, kTrackWidth / 2), //fr
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2), //bl
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2)); //br


        /*
         * 
         * 
         * new Translation2d(-kWheelBase / 2, -kTrackWidth / 2), //br
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2), //fr
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2), //bl
                new Translation2d(kWheelBase / 2, kTrackWidth / 2)); //fl

         */

                                                               // Driving Motor Ports
        public static final int kFrontLeftDriveMotorPort = 3;  // Front Left 
        public static final int kFrontRightDriveMotorPort = 40; // Front Right
        public static final int kBackRightDriveMotorPort = 30;  // Back Right
        public static final int kBackLeftDriveMotorPort = 10;   // Back Left

                                                                // Turning Motor Ports
        public static final int kFrontLeftTurningMotorPort = 4; // Front Left
        public static final int kFrontRightTurningMotorPort = 41;// Front Right
        public static final int kBackRightTurningMotorPort = 31;// Back Right
        public static final int kBackLeftTurningMotorPort = 9;  // Back Left

        // Encoder on NEO turning
        public static final boolean kFrontLeftTurningEncoderReversed = true;
        public static final boolean kBackLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kBackRightTurningEncoderReversed = true;

        // Encoder for NEO drive
        public static final boolean kFrontLeftDriveEncoderReversed = true;
        public static final boolean kBackLeftDriveEncoderReversed = true;
        public static final boolean kFrontRightDriveEncoderReversed = true; //
        public static final boolean kBackRightDriveEncoderReversed = true;  //

        // -------> ABE <-------- //
        public static final int kFrontLeftDriveAbsoluteEncoderPort = 44;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 11;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 33;
        public static final int kBackRightDriveAbsoluteEncoderPort = 22;
        // -------> ABE <-------- //

        // Absolute encoders reversed
        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

                                        // Need to update values for our specific magnetic fields
                                        // NOT IN RADIANS!
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = Math.PI/2;//0.98 * 2 * Math.PI;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = Math.PI/2;        // 1.04719      //1.04719
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = Math.PI/2;//(0.0141+.25) * 2 * Math.PI;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad =  Math.PI/2;//0.2577 * 2 * Math.PI;

        public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 8; //1.75
       // public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 5; //1.75
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 4; //2

        public static final double kPThetaController = 0.001;
        public static final double kIThetaController = 0.0;
        public static final double kDThetaController = 0.00;
        
        public static final double kMaxDriveMotorTemp = 33.0;

    }

    // Autonomous
    public static final class AutoConstants {

        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 8;
        public static final double kMaxAngularSpeedRadiansPerSecond = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 15;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 1;
        public static final double kPYController = 1;

        public static final double kPThetaController = 0.01;
        public static final double kIThetaController = 0.0;
        public static final double kDThetaController = 0.005;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = 
        new TrapezoidProfile.Constraints(
        5,
        5);
    
    }

    // Input and Output 
    public static final class IOConstants {

        public static final double kDeadband = 0.1;

        public static final int kLeftJoystick = 0;
        public static final int kRightJoystick = 1;
        //public static final int kXboxController = 3;

        public static final int kFieldOrientedButton = 3;
        public static final int kZeroHeadingButton = 2;
        public static final int kRotatorButton = 3;
        public static final double kTransmitterOffset = 1.429;
    }

    

    //public static final class ShooterConstants{

        //public static final int kLeftShooterMotorPort = 19;
        //public static final int kRightShooterMotorPort = 17;
        //public static final int kLeftHoldMotorPort = 7;
        //public static final int kRightHoldMotorPort = 2;   

//        public static final double kP = 0.1;

   // }
    // public static final class AngleConstants{

      //  public static final int kAngleMotorPort = 18;
        
      //  public static final double kP = 0.1;

   // }

    //public static final class IntakeConstants{

     //   public static final int kIntakeMotorPort = 6;

    //}

    public static final class ArmConstants{

        public static final int kArmMotorPort = 8;
        public static final int kArmRotMotorPort = 2;

    }

    public static final class ConveyorConstants{

        public static final int kConveyorMotorPort = 7;

    }

    public static final class ElevatorConstants{

        public static final int kElevatorMotorPort = 6;

    }

public static final class usbPortConstants {
    public static final int kCamera = 0;
}

    public static final class visionConstants {
        public static final int[] kCameraResolution = new int[] { 320, 240};
        public static final int kCameraFPS = 15;
    }


    }


