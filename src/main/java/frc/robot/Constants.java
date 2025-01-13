package frc.robot;

import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class Constants {
  public static final class PhysicalConstants {
    public static final double kRobotMassPounds = 100;

    public static final double kBumperLength = 32;

    public static final Translation2d kFrontLeftLocationInches = new Translation2d(15, 15);
    public static final Translation2d kFrontRightLocationInches = new Translation2d(15, -15);
    public static final Translation2d kBackLeftLocationInches = new Translation2d(-15, 15);
    public static final Translation2d kBackRightLocationInches = new Translation2d(-15, -15);

    public static final double kMaxSpeed = 3.0;
    public static final double kMaxAngularSpeed = Math.PI;

    public static final double kModuleMaxAngularVelocity = kMaxAngularSpeed;
    public static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared
  }
  public static final class SwerveConstants {
    public static final PIDController kDrivePIDController = new PIDController(1, 0, 0); // CHANGE THIS --------------------------------------------------
    public static final ProfiledPIDController kTurningPIDController = new ProfiledPIDController(1, 0, 0, 
      new TrapezoidProfile.Constraints(
        PhysicalConstants.kModuleMaxAngularVelocity, PhysicalConstants.kModuleMaxAngularAcceleration
      ));

    public static final SimpleMotorFeedforward kDriveFeedforward = new SimpleMotorFeedforward(1, 3);
    public static final SimpleMotorFeedforward kTurnFeedforward = new SimpleMotorFeedforward(1, 0.5);

    public static final double kWheelRadius = 0.0508;
    public static final int kEncoderResolution = 4096;

    public static final double kDtSeconds = 0.02;
  }

  public static final class GyroConstants {
    public static final NavXComType kComType = NavXComType.kI2C; // CHANGE IF NECCESARY --------------------------------------------------------------------------
  }
  
  public static final class ControllerConstants {
    public static final double kFlightStickDeadband = 0.05;
  }
}