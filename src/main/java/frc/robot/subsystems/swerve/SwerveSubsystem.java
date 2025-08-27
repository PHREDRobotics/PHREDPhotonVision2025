package frc.robot.subsystems.swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;

/**
 * Subsystem for controlling swerve
 */
public class SwerveSubsystem extends SubsystemBase {
  private final SwerveModule m_frontLeft;
  private final SwerveModule m_frontRight;
  private final SwerveModule m_backLeft;
  private final SwerveModule m_backRight;

  private final AHRS m_gyro;

  private final SwerveDrivePoseEstimator m_poseEstimator;

  RobotConfig config;

  private final StructArrayPublisher<SwerveModuleState> publisher;

  /**
   * Creates a new swerve subsystem
   * 
   * @param photonVision used for pose estimation
   */
  public SwerveSubsystem() {
    m_frontLeft = new SwerveModule(Constants.SwerveConstants.kFrontLeftDriveMotorCANId,
        Constants.SwerveConstants.kFrontLeftTurnMotorCANId,
        Configs.FrontLeftConfig.drivingConfig,
        Configs.FrontLeftConfig.turningConfig);
    m_frontRight = new SwerveModule(Constants.SwerveConstants.kFrontRightDriveMotorCANId,
        Constants.SwerveConstants.kFrontRightTurnMotorCANId,
        Configs.FrontRightConfig.drivingConfig,
        Configs.FrontRightConfig.turningConfig);
    m_backLeft = new SwerveModule(Constants.SwerveConstants.kBackLeftDriveMotorCANId,
        Constants.SwerveConstants.kBackLeftTurnMotorCANId,
        Configs.BackLeftConfig.drivingConfig,
        Configs.BackLeftConfig.turningConfig);
    m_backRight = new SwerveModule(Constants.SwerveConstants.kBackRightDriveMotorCANId,
        Constants.SwerveConstants.kBackRightTurnMotorCANId,
        Configs.BackRightConfig.drivingConfig,
        Configs.BackRightConfig.turningConfig);

    m_gyro = new AHRS(Constants.GyroConstants.kComType);

    m_gyro.reset();

    publisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("/SwerveStates", SwerveModuleState.struct).publish();

    m_poseEstimator = new SwerveDrivePoseEstimator(Constants.SwerveConstants.kKinematics, getRotation(),
        getModulePositions(), new Pose2d(), Constants.SwerveConstants.kStateStdDevs,
        Constants.SwerveConstants.kVisionStdDevs);

    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
    }

    AutoBuilder.configure(
        () -> getPose(),
        (pose) -> resetOdometry(pose),
        () -> getSpeeds(false),
        (speeds, feedforwards) -> drive(
            speeds,
            false),
        new PPHolonomicDriveController(
            new PIDConstants(0.6, 0.2, 0),
            new PIDConstants(1.0, 0, 0)),
        config,
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this);
  }

  /**
   * @param xSpeed
   * @param ySpeed
   * @param rot
   * @param fieldOriented
   */
  public void drive(double xSpeed, double ySpeed, double rot,
      boolean fieldOriented) {
    var swerveModuleStates = Constants.SwerveConstants.kKinematics.toSwerveModuleStates(
        ChassisSpeeds.discretize(
            fieldOriented
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed
                        * Constants.PhysicalConstants.kMaxSpeed,
                    ySpeed
                        * Constants.PhysicalConstants.kMaxSpeed,
                    rot
                        * Constants.PhysicalConstants.kMaxAngularSpeed,
                    m_gyro.getRotation2d())
                : new ChassisSpeeds(
                    xSpeed
                        * Constants.PhysicalConstants.kMaxSpeed,
                    ySpeed
                        * Constants.PhysicalConstants.kMaxSpeed,
                    rot
                        * Constants.PhysicalConstants.kMaxAngularSpeed),
            Constants.SwerveConstants.kDtSeconds));

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.PhysicalConstants.kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);

    publisher.set(swerveModuleStates);
  }

  public void drive(ChassisSpeeds speeds, boolean fieldOriented) {
    var swerveModuleStates = Constants.SwerveConstants.kKinematics.toSwerveModuleStates(
        ChassisSpeeds.discretize(
            fieldOriented
                ? ChassisSpeeds.fromFieldRelativeSpeeds(speeds,
                    m_gyro.getRotation2d())
                : speeds,
            Constants.SwerveConstants.kDtSeconds));

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.PhysicalConstants.kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);

    publisher.set(swerveModuleStates);
  }

  public Rotation2d getRotation() {
    return m_gyro.getRotation2d();
  }

  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  public void resetOdometry(Pose2d pose) {
    m_poseEstimator.resetPose(pose);
  }

  public void resetGyro() {
    m_gyro.reset();
  }

  public void updateOdometry() {
    m_poseEstimator.update(getRotation(), getModulePositions());
  }

  public void addVisionMeasurement(Pose2d measurement, double timestamp) {
    m_poseEstimator.addVisionMeasurement(measurement, timestamp);
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_backLeft.getPosition(),
        m_backRight.getPosition()
    };
  }

  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_backLeft.getState(),
        m_backRight.getState() };
  }

  public ChassisSpeeds getSpeeds(boolean fieldRelative) {
    ChassisSpeeds robotRelativeSpeeds = Constants.SwerveConstants.kKinematics.toChassisSpeeds(getModuleStates());

    if (fieldRelative) {
      return robotRelativeSpeeds;
    }

    return ChassisSpeeds.fromRobotRelativeSpeeds(
        robotRelativeSpeeds, getPose().getRotation());
  }

  public void periodic() {
    updateOdometry();

    SmartDashboard.putNumber("F.L. Drive motor temp", m_frontLeft.getDriveTemp());

    SmartDashboard.putNumber("Swerve/FrontLeft/Speed", m_frontLeft.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("Swerve/FrontLeft/Angle", m_frontLeft.getPosition().angle.getDegrees());
    SmartDashboard.putNumber("Swerve/FrontRight/Speed", m_frontRight.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("Swerve/FrontRight/Angle", m_frontRight.getPosition().angle.getDegrees());
    SmartDashboard.putNumber("Swerve/BackLeft/Speed", m_backLeft.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("Swerve/BackLeft/Angle", m_backLeft.getPosition().angle.getDegrees());
    SmartDashboard.putNumber("Swerve/BackRight/Speed", m_backRight.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("Swerve/BackRight/Angle", m_backRight.getPosition().angle.getDegrees());

    SmartDashboard.putString("Speeds", getSpeeds(true).toString());
    SmartDashboard.putString("States/FL", getModuleStates()[0].toString());
    SmartDashboard.putString("States/FR", getModuleStates()[1].toString());
    SmartDashboard.putString("States/BL", getModuleStates()[2].toString());
    SmartDashboard.putString("States/BR", getModuleStates()[3].toString());

    SmartDashboard.updateValues();
  }

  @Override
  public void simulationPeriodic() {
  }
}
