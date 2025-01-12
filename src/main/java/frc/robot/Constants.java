package frc.robot;

import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class Constants {
  public static final class SwerveConstants {
    public static final double kMaxSpeed = 3.0;
    public static final double kMaxAngularSpeed = Math.PI;

    private static final double kModuleMaxAngularVelocity = kMaxAngularSpeed;
    private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared

    public static final Translation2d kFrontLeftLocation = new Translation2d(0.381, 0.381);
    public static final Translation2d kFrontRightLocation = new Translation2d(0.381, -0.381);
    public static final Translation2d kBackLeftLocation = new Translation2d(-0.381, 0.381);
    public static final Translation2d kBackRightLocation = new Translation2d(-0.381, -0.381);

    public static final PIDController kDrivePIDController = new PIDController(1, 0, 0); // CHANGE THIS --------------------------------------------------
    public static final ProfiledPIDController kTurningPIDController = new ProfiledPIDController(1, 0, 0, 
                                                                                                new TrapezoidProfile.Constraints(
                                                                                                  kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration
                                                                                                ));

    public static final SimpleMotorFeedforward kDriveFeedforward = new SimpleMotorFeedforward(1, 3);
    public static final SimpleMotorFeedforward kTurnFeedforward = new SimpleMotorFeedforward(1, 0.5);

    public static final double kWheelRadius = 0.0508;
    public static final int kEncoderResolution = 4096;
  }

  public static final NavXComType kComType = NavXComType.kI2C; // CHANGE IF NECCESARY --------------------------------------------------------------------------
}