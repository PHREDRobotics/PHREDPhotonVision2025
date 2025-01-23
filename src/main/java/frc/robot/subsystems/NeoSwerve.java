package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class NeoSwerve extends SubsystemBase implements SwerveDrive {
    private final Translation2d m_frontLeftLocationMeters = new Translation2d(
        Units.inchesToMeters(Constants.PhysicalConstants.kFrontLeftLocationInches.getX()),
        Units.inchesToMeters(Constants.PhysicalConstants.kFrontLeftLocationInches.getY()));
    private final Translation2d m_frontRightLocationMeters = new Translation2d(
        Units.inchesToMeters(Constants.PhysicalConstants.kFrontRightLocationInches.getX()),
        Units.inchesToMeters(Constants.PhysicalConstants.kFrontRightLocationInches.getY()));
    private final Translation2d m_backLeftLocationMeters = new Translation2d(
        Units.inchesToMeters(Constants.PhysicalConstants.kBackLeftLocationInches.getX()),
        Units.inchesToMeters(Constants.PhysicalConstants.kBackLeftLocationInches.getY()));
    private final Translation2d m_backRightLocationMeters = new Translation2d(
        Units.inchesToMeters(Constants.PhysicalConstants.kBackRightLocationInches.getX()),
        Units.inchesToMeters(Constants.PhysicalConstants.kBackRightLocationInches.getY()));

    private final SparkMax m_frontLeftDriveDataCollector = new SparkMax(
        Constants.SwerveConstants.kFrontLeftDriveMotorCANId, MotorType.kBrushless); // This is specifically to get information like temperature from the motors

    private final SwerveModule m_frontLeft = new SwerveModule(
        Constants.SwerveConstants.kFrontLeftDriveMotorCANId, Constants.SwerveConstants.kFrontLeftTurnMotorCANId);
    private final SwerveModule m_frontRight = new SwerveModule(
        Constants.SwerveConstants.kFrontRightDriveMotorCANId, Constants.SwerveConstants.kFrontRightTurnMotorCANId);
    private final SwerveModule m_backLeft = new SwerveModule(
        Constants.SwerveConstants.kBackLeftDriveMotorCANId, Constants.SwerveConstants.kBackLeftTurnMotorCANId);
    private final SwerveModule m_backRight = new SwerveModule(
        Constants.SwerveConstants.kBackRightDriveMotorCANId, Constants.SwerveConstants.kBackRightTurnMotorCANId);

    private final AHRS m_gyro = new AHRS(Constants.GyroConstants.kComType);

    private final SwerveDriveKinematics m_kinematics =
        new SwerveDriveKinematics(
            m_frontLeftLocationMeters, m_frontRightLocationMeters, m_backLeftLocationMeters, m_backRightLocationMeters);

    private final SwerveDriveOdometry m_odometry =
        new SwerveDriveOdometry(
            m_kinematics,
            m_gyro.getRotation2d(),
            new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_backLeft.getPosition(),
                m_backRight.getPosition()});

    public NeoSwerve() {
        m_gyro.reset();
    }

    @Override
    public void drive(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rot, BooleanSupplier fieldOriented) {
        var swerveModuleStates =
            m_kinematics.toSwerveModuleStates(
                ChassisSpeeds.discretize(
                    fieldOriented.getAsBoolean()
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(
                            xSpeed.getAsDouble(), ySpeed.getAsDouble(), rot.getAsDouble(), m_gyro.getRotation2d())
                        : new ChassisSpeeds(xSpeed.getAsDouble(), ySpeed.getAsDouble(), rot.getAsDouble()),
                    Constants.SwerveConstants.kDtSeconds));

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.PhysicalConstants.kMaxSpeed);
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_backLeft.setDesiredState(swerveModuleStates[2]);
        m_backRight.setDesiredState(swerveModuleStates[3]);
    }

    @Override
    public void drive(ChassisSpeeds speeds, BooleanSupplier fieldOriented) {
        var swerveModuleStates =
            m_kinematics.toSwerveModuleStates(
                ChassisSpeeds.discretize(
                    fieldOriented.getAsBoolean()
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(speeds, m_gyro.getRotation2d())
                        : speeds,
                    Constants.SwerveConstants.kDtSeconds));
        
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.PhysicalConstants.kMaxSpeed);
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_backLeft.setDesiredState(swerveModuleStates[2]);
        m_backRight.setDesiredState(swerveModuleStates[3]);
    }

    @Override
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    @Override
    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPose(pose);
    }

    @Override
    public void updateOdometry() {
        m_odometry.update(
            m_gyro.getRotation2d(),
            new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_backLeft.getPosition(),
                m_backRight.getPosition()
            });
    }

    @Override
    public void periodic() {
        updateOdometry();
    }

    @Override
    public void simulationPeriodic() {
        updateOdometry();

        SmartDashboard.putNumber("F.R. Drive motor temp", m_frontLeftDriveDataCollector.getMotorTemperature());
    }
}