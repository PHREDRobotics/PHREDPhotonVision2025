package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveDriveSimulation m_simulatedSwerve;
    private final Field2d m_field;

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
    
    private final SwerveModule m_frontLeft;
    private final SwerveModule m_frontRight;
    private final SwerveModule m_backLeft;
    private final SwerveModule m_backRight;

    private final AHRS m_gyro = new AHRS(Constants.GyroConstants.kComType);

    private final SwerveDriveKinematics m_kinematics =
        new SwerveDriveKinematics(
            m_frontLeftLocationMeters, m_frontRightLocationMeters, m_backLeftLocationMeters, m_backRightLocationMeters);
    
    private final SwerveDriveOdometry m_odometry;

    RobotConfig config;

    public SwerveSubsystem() {
        if (RobotBase.isReal()) {
            m_frontLeft = new SparkSwerveModule(Constants.SwerveConstants.kFrontLeftDriveMotorCANId, Constants.SwerveConstants.kFrontLeftTurnMotorCANId);
            m_frontRight = new SparkSwerveModule(Constants.SwerveConstants.kFrontRightDriveMotorCANId, Constants.SwerveConstants.kFrontRightTurnMotorCANId);
            m_backLeft = new SparkSwerveModule(Constants.SwerveConstants.kBackLeftDriveMotorCANId, Constants.SwerveConstants.kBackRightTurnMotorCANId);
            m_backRight = new SparkSwerveModule(Constants.SwerveConstants.kBackRightDriveMotorCANId, Constants.SwerveConstants.kBackRightTurnMotorCANId);
        } else {
            m_frontLeft = new SparkSwerveModuleSim(Constants.SwerveConstants.kFrontLeftDriveMotorCANId, Constants.SwerveConstants.kFrontLeftTurnMotorCANId);
            m_frontRight = new SparkSwerveModuleSim(Constants.SwerveConstants.kFrontRightDriveMotorCANId, Constants.SwerveConstants.kFrontRightTurnMotorCANId);
            m_backLeft = new SparkSwerveModuleSim(Constants.SwerveConstants.kBackLeftDriveMotorCANId, Constants.SwerveConstants.kBackLeftTurnMotorCANId);
            m_backRight = new SparkSwerveModuleSim(Constants.SwerveConstants.kBackRightDriveMotorCANId, Constants.SwerveConstants.kBackRightTurnMotorCANId);
        }

        final DriveTrainSimulationConfig drivetrainConfig = new DriveTrainSimulationConfig(
            Pounds.of(Constants.PhysicalConstants.kRobotMassPounds),
            Inches.of(Constants.PhysicalConstants.kBumperLength),
            Inches.of(Constants.PhysicalConstants.kBumperLength),
            Inches.of(Constants.PhysicalConstants.kFrontLeftLocationInches.getX() * 2),
            Inches.of(Constants.PhysicalConstants.kFrontLeftLocationInches.getY() * 2),
            () -> COTS.ofMark4i(DCMotor.getNEO(1), DCMotor.getNEO(1), 1, 1).get(),
            () -> COTS.ofNav2X().get());

        this.m_simulatedSwerve = new SwerveDriveSimulation(drivetrainConfig, new Pose2d(8.1, 7, new Rotation2d(Math.PI)));
        
        m_field = new Field2d();
        SmartDashboard.putData("Sim field", m_field);

        m_gyro.reset();

        m_odometry = 
            new SwerveDriveOdometry(
                m_kinematics,
                m_gyro.getRotation2d(),
                new SwerveModulePosition[] {
                    m_frontLeft.getPosition(),
                    m_frontRight.getPosition(),
                    m_backLeft.getPosition(),
                    m_backRight.getPosition()});

        try{
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
                new PIDConstants(1.0, 0, 0)
            ),
            config,
            () -> {
              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Blue;
              }
              return false;
            },
            this
        );

        SimulatedArena.getInstance().addDriveTrainSimulation(m_simulatedSwerve);
    }

    public void drive(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rot, BooleanSupplier fieldOriented) {
        var swerveModuleSpeeds =
            ChassisSpeeds.discretize(
                fieldOriented.getAsBoolean()
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed.getAsDouble() * Constants.PhysicalConstants.kMaxSpeed,
                        ySpeed.getAsDouble() * Constants.PhysicalConstants.kMaxSpeed,
                        rot.getAsDouble() * Constants.PhysicalConstants.kMaxAngularSpeed,
                        m_gyro.getRotation2d())
                    : new ChassisSpeeds(
                    xSpeed.getAsDouble() * Constants.PhysicalConstants.kMaxSpeed,
                    ySpeed.getAsDouble() * Constants.PhysicalConstants.kMaxSpeed,
                    rot.getAsDouble() * Constants.PhysicalConstants.kMaxAngularSpeed),
                Constants.SwerveConstants.kDtSeconds);
        
        var swerveModuleStates = m_kinematics.toSwerveModuleStates(swerveModuleSpeeds);

        m_simulatedSwerve.setRobotSpeeds(swerveModuleSpeeds);
        
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.PhysicalConstants.kMaxSpeed);
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_backLeft.setDesiredState(swerveModuleStates[2]);
        m_backRight.setDesiredState(swerveModuleStates[3]);
    }

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

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPose(pose);
    }

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

    public ChassisSpeeds getSpeeds(boolean fieldRelative) {
        ChassisSpeeds robotRelativeSpeeds = m_kinematics.toChassisSpeeds(new SwerveModuleState[] {
            m_frontLeft.getState(),
            m_frontRight.getState(),
            m_backLeft.getState(),
            m_backRight.getState()});

        if (fieldRelative) {
            return robotRelativeSpeeds;
        }

        return ChassisSpeeds.fromRobotRelativeSpeeds(
                robotRelativeSpeeds, getPose().getRotation());
    }

    public void periodic() {
        updateOdometry();

        m_simulatedSwerve.simulationSubTick();

        SmartDashboard.putNumber("F.L. Drive motor temp", m_frontLeft.getDriveTemp());

        SmartDashboard.putNumber("Swerve/FrontLeft/Speed", m_frontLeft.getState().speedMetersPerSecond);
        SmartDashboard.putNumber("Swerve/FrontLeft/Angle", m_frontLeft.getPosition().angle.getDegrees());
        SmartDashboard.putNumber("Swerve/FrontRight/Speed", m_frontRight.getState().speedMetersPerSecond);
        SmartDashboard.putNumber("Swerve/FrontRight/Angle", m_frontRight.getPosition().angle.getDegrees());
        SmartDashboard.putNumber("Swerve/BackLeft/Speed", m_backLeft.getState().speedMetersPerSecond);
        SmartDashboard.putNumber("Swerve/BackLeft/Angle", m_backLeft.getPosition().angle.getDegrees());
        SmartDashboard.putNumber("Swerve/BackRight/Speed", m_backRight.getState().speedMetersPerSecond);
        SmartDashboard.putNumber("Swerve/BackRight/Angle", m_backRight.getPosition().angle.getDegrees());

        SmartDashboard.updateValues();

        m_field.setRobotPose(m_simulatedSwerve.getSimulatedDriveTrainPose());
    }

    @Override
    public void simulationPeriodic() {
        updateOdometry();
    }
}