package frc.robot.subsystems.swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.photonvision.EstimatedRobotPose;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.subsystems.vision.VisionSubsystem;

/**
 * Subsystem for controlling swerve
 */
public class SwerveSubsystem extends SubsystemBase {
  VisionSubsystem photonVision;

  private final SwerveModule m_frontLeft;
  private final SwerveModule m_frontRight;
  private final SwerveModule m_backLeft;
  private final SwerveModule m_backRight;

  private final AHRS m_gyro;

  private final SwerveDriveOdometry m_odometry;

  RobotConfig config;

  private final Field2d m_poseEstimatorField;

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

    m_odometry = new SwerveDriveOdometry(Constants.SwerveConstants.kKinematics, getPose().getRotation(),
        getModulePositions(), new Pose2d());

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
            () -> false),
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

    m_poseEstimatorField = new Field2d();
    SmartDashboard.putData("Pose estimation field", m_poseEstimatorField);
  }

  /**
   * @param xSpeed
   * @param ySpeed
   * @param rot
   * @param fieldOriented
   */
  public void drive(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rot,
      BooleanSupplier fieldOriented) {
    var swerveModuleStates = Constants.SwerveConstants.kKinematics.toSwerveModuleStates(
        ChassisSpeeds.discretize(
            fieldOriented.getAsBoolean()
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed.getAsDouble()
                        * Constants.PhysicalConstants.kMaxSpeed,
                    ySpeed.getAsDouble()
                        * Constants.PhysicalConstants.kMaxSpeed,
                    rot.getAsDouble()
                        * Constants.PhysicalConstants.kMaxAngularSpeed,
                    m_gyro.getRotation2d())
                : new ChassisSpeeds(
                    xSpeed.getAsDouble()
                        * Constants.PhysicalConstants.kMaxSpeed,
                    ySpeed.getAsDouble()
                        * Constants.PhysicalConstants.kMaxSpeed,
                    rot.getAsDouble()
                        * Constants.PhysicalConstants.kMaxAngularSpeed),
            Constants.SwerveConstants.kDtSeconds));

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.PhysicalConstants.kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);

    SmartDashboard.putString("Input/FL", swerveModuleStates[0].toString());
    SmartDashboard.putString("Input/FR", swerveModuleStates[1].toString());
    SmartDashboard.putString("Input/BL", swerveModuleStates[2].toString());
    SmartDashboard.putString("Input/BR", swerveModuleStates[3].toString());

    SmartDashboard.putNumber("Rot", rot.getAsDouble());

    publisher.set(swerveModuleStates);
  }

  public void drive(ChassisSpeeds speeds, BooleanSupplier fieldOriented) {
    var swerveModuleStates = Constants.SwerveConstants.kKinematics.toSwerveModuleStates(
        ChassisSpeeds.discretize(
            fieldOriented.getAsBoolean()
                ? ChassisSpeeds.fromFieldRelativeSpeeds(speeds,
                    m_gyro.getRotation2d())
                : speeds,
            Constants.SwerveConstants.kDtSeconds));

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.PhysicalConstants.kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPose(pose);
  }

  public void resetGyro() {
    m_gyro.reset();
  }

  public void updateOdometry() {
    m_odometry.update(
        m_gyro.getRotation2d(),
        getModulePositions());

    EstimatedRobotPose currentPose;

    if (photonVision.getEstimatedGlobalPose().isPresent()) {
      currentPose = photonVision.getEstimatedGlobalPose().get();

      
    }

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

    m_poseEstimatorField.setRobotPose(getPose());

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
