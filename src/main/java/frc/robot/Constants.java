package frc.robot;

import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController.Button;

public class Constants {
  public static final class PhysicalConstants {
    public static final double kRobotMassPounds = 100;

    public static final double kBumperLength = 32;
    public static final double kTrackLength = 24;

    public static final Translation2d kFrontLeftLocationInches = new Translation2d(-21, 13.5);
    public static final Translation2d kFrontRightLocationInches = new Translation2d(3, 13.5);
    public static final Translation2d kBackLeftLocationInches = new Translation2d(-21, -10.5);
    public static final Translation2d kBackRightLocationInches = new Translation2d(3, -10.5);

    public static final double kMaxSpeed = 2 * Math.floor(Math.PI); // 6
    public static final double kMaxAngularSpeed = 6;

    public static final double kModuleMaxAngularVelocity = kMaxAngularSpeed;
    public static final double kModuleMaxAngularAcceleration = kModuleMaxAngularVelocity * kModuleMaxAngularVelocity; // radians per second squared
  }
  public static final class SwerveConstants {
    public static final double kWheelRadius = 0.0508;
    public static final int kEncoderResolution = 4096;

    public static final double kDtSeconds = 0.02;

    public static final int kBackLeftDriveMotorCANId = 26;
    public static final int kFrontLeftDriveMotorCANId = 21;
    public static final int kFrontRightDriveMotorCANId = 11;
    public static final int kBackRightDriveMotorCANId = 16;

    public static final int kBackLeftTurnMotorCANId = 27;
    public static final int kFrontLeftTurnMotorCANId = 22;
    public static final int kFrontRightTurnMotorCANId = 12;
    public static final int kBackRightTurnMotorCANId = 17;

    public static final double kDrivingMotorReduction = 8;
    public static final double kTurningMotorReduction = 8;

    public static final double kDriveAccelerationLimiter = 5;
    public static final double kRotationAccelerationLimiter = 7.5;


    public static final Translation2d kFrontLeftLocationMeters = new Translation2d(
            Units.inchesToMeters(Constants.PhysicalConstants.kFrontLeftLocationInches.getX()),
            Units.inchesToMeters(Constants.PhysicalConstants.kFrontLeftLocationInches.getY()));
    public static final Translation2d kFrontRightLocationMeters = new Translation2d(
            Units.inchesToMeters(Constants.PhysicalConstants.kFrontRightLocationInches.getX()),
            Units.inchesToMeters(Constants.PhysicalConstants.kFrontRightLocationInches.getY()));
    public static final Translation2d kBackLeftLocationMeters = new Translation2d(
            Units.inchesToMeters(Constants.PhysicalConstants.kBackLeftLocationInches.getX()),
            Units.inchesToMeters(Constants.PhysicalConstants.kBackLeftLocationInches.getY()));
    public static final Translation2d kBackRightLocationMeters = new Translation2d(
            Units.inchesToMeters(Constants.PhysicalConstants.kBackRightLocationInches.getX()),
            Units.inchesToMeters(Constants.PhysicalConstants.kBackRightLocationInches.getY()));
    
    public static final SwerveDriveKinematics kKinematics = new SwerveDriveKinematics(
            Constants.SwerveConstants.kFrontLeftLocationMeters, Constants.SwerveConstants.kFrontRightLocationMeters, 
            Constants.SwerveConstants.kBackLeftLocationMeters, Constants.SwerveConstants.kBackRightLocationMeters);
  }

  public static final class GyroConstants {
    public static final NavXComType kComType = NavXComType.kMXP_SPI;
  }

  public static final class ControllerConstants {
    public static final double kFlightStickXDeadband = 0.25;
    public static final double kFlightStickYDeadband = 0.1;
    public static final double kFlightStickZDeadband = 0.15;

    public static final double kThrottleMultiplier = 1;
  }

  public static final class VisionConstants {
    public static final double kLimelightMountAngleDegrees = 0.0; // 33.0?
    public static final double kLimelightLensHeightInches = 0.0; // 32.0? - last year
    public static final double kAmpOrSourceHeightInches = 48.5;
    public static final double kSpeakerHeightInches = 51.0 + 7.0 / 8.0;
    public static final double kStageHeightInches = 47.5;
    public static final double kAreaToCentimeters = 150;


    public static final double kPdrive = 0.1;
    public static final double kIdrive = 0;
    public static final double kDdrive = 0;

    public static final double kPstrafe = 0.08;
    public static final double kIstrafe = 0;
    public static final double kDstrafe = 0;


  }

    public static final class CoralConstants {
    
      public static final int kCoralSparkMaxCanID = 6;
      public static final double kCoralIntakeSpeed = -0.5;
      public static final double kCoralIntakeTime = 1.0;
      public static final double kCoralOuttakeSpeed = 0.5;
      public static final double kCoralOuttakeTime = 1.0;
      public static final boolean test = false;
      }    

      public static final class AlgaeConstants {
        public static final int kLeftAlgaeControllerPort = 8;
        public static final int kRightAlgaeControllerPort = 9;
        public static final int kXBtn = Button.kX.value;
        public static final double kAlgaeSpeed = 0.75;
        public static final double kAlgaeTime = 2;
      }

      public static final class PneumaticsConstants {
        public static final int kPneumaticsCANID = 40;
        public static final int kSolenoidInput = 14;
        public static final int kSolenoidOutput = 13;
      }
}