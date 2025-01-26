package frc.robot;

import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;

public class Constants {
  public static final class PhysicalConstants {
    public static final double kRobotMassPounds = 100;

    public static final double kBumperLength = 32;

    public static final Translation2d kFrontLeftLocationInches = new Translation2d(15, 15);
    public static final Translation2d kFrontRightLocationInches = new Translation2d(15, -15);
    public static final Translation2d kBackLeftLocationInches = new Translation2d(-15, 15);
    public static final Translation2d kBackRightLocationInches = new Translation2d(-15, -15);

    public static final double kMaxSpeed = 6; // 3
    public static final double kMaxAngularSpeed = 6;

    public static final double kModuleMaxAngularVelocity = kMaxAngularSpeed;
    public static final double kModuleMaxAngularAcceleration = kModuleMaxAngularVelocity * kModuleMaxAngularVelocity; // radians per second squared
  }
  public static final class SwerveConstants {
    public static final PIDController kDrivePIDController = new PIDController(1, 0, 0); // CHANGE THIS --------------------------------------------------
    //public static final PIDController kTurningPIDController = new PIDController(1, 0, 0);
    //public static final ProfiledPIDController kTurningPIDController = new ProfiledPIDController(0.9, 0.6, 0.5,
    //  new TrapezoidProfile.Constraints(
    //    PhysicalConstants.kModuleMaxAngularVelocity, PhysicalConstants.kModuleMaxAngularAcceleration
    //  ));

    public static final SimpleMotorFeedforward kDriveFeedforward = new SimpleMotorFeedforward(1, 3);
    public static final SimpleMotorFeedforward kTurnFeedforward = new SimpleMotorFeedforward(1, 0.5);

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

    public static final double kDrivingMotorReduction = 4;
  }

  public static final class GyroConstants {
    public static final NavXComType kComType = NavXComType.kMXP_SPI;
  }

  public static final class ControllerConstants {
    public static final double kFlightStickXDeadband = 0.25;
    public static final double kFlightStickYDeadband = 0.1;
    public static final double kFlightStickZDeadband = 0.15;
  }
}