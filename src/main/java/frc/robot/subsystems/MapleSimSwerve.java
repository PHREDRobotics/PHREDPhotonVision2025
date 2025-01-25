package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Simulation implementation of {@link SwerveDrive} using maple-sim
 */
public class MapleSimSwerve extends SubsystemBase implements SwerveDrive {
    private final SelfControlledSwerveDriveSimulation m_simulatedSwerve;
    private final Field2d field2d;

    private final Field2d odometry_field2d;

    private final StructArrayPublisher<SwerveModuleState> publisher;

    private final SwerveDriveOdometry m_odometry;
    private final SwerveDriveKinematics m_kinematics;

    RobotConfig config;

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

    public MapleSimSwerve() {
        final DriveTrainSimulationConfig drivetrainConfig = new DriveTrainSimulationConfig(
            Pounds.of(Constants.PhysicalConstants.kRobotMassPounds),
            Inches.of(Constants.PhysicalConstants.kBumperLength),
            Inches.of(Constants.PhysicalConstants.kBumperLength),
            Inches.of(Constants.PhysicalConstants.kFrontLeftLocationInches.getX() * 2),
            Inches.of(Constants.PhysicalConstants.kFrontLeftLocationInches.getY() * 2),
            () -> COTS.ofMark4i(DCMotor.getNEO(1), DCMotor.getNEO(1), 1, 1).get(),
            () -> COTS.ofNav2X().get());

        this.m_simulatedSwerve = new SelfControlledSwerveDriveSimulation(
            new SwerveDriveSimulation(drivetrainConfig, new Pose2d(8.1, 7, new Rotation2d(Math.PI))));
        
        m_kinematics = new SwerveDriveKinematics(
            m_frontLeftLocationMeters, m_frontRightLocationMeters, m_backLeftLocationMeters, m_backRightLocationMeters);  

        m_odometry = new SwerveDriveOdometry(
            m_kinematics, m_simulatedSwerve.getActualPoseInSimulationWorld().getRotation(),
            m_simulatedSwerve.getLatestModulePositions());
        
        SimulatedArena.getInstance().addDriveTrainSimulation(m_simulatedSwerve.getDriveTrainSimulation());

        field2d = new Field2d();
        SmartDashboard.putData("Sim field", field2d);

        odometry_field2d = new Field2d();
        SmartDashboard.putData("Odometry readings", odometry_field2d);

        publisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("/SwerveStates", SwerveModuleState.struct).publish();     

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
                    new PIDConstants(0.6, 0.2, 0.0),
                    new PIDConstants(1.0, 0.0, 0.0)
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
    }

    @Override
    public void drive(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rot, BooleanSupplier fieldOriented) {
        this.m_simulatedSwerve.runChassisSpeeds(
            new ChassisSpeeds(-xSpeed.getAsDouble(), -ySpeed.getAsDouble(), -rot.getAsDouble()),
            new Translation2d(),
            fieldOriented.getAsBoolean(),
            true);
    }

    @Override
    public void drive(ChassisSpeeds speeds, BooleanSupplier fieldOriented) {
        this.m_simulatedSwerve.runChassisSpeeds(
            speeds,
            new Translation2d(),
            fieldOriented.getAsBoolean(),
            true);
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
            m_simulatedSwerve.getOdometryEstimatedPose().getRotation(),
            m_simulatedSwerve.getLatestModulePositions());
    }

    @Override
    public ChassisSpeeds getSpeeds(boolean fieldRelative) {
        if (fieldRelative) {
            return m_simulatedSwerve.getMeasuredSpeedsFieldRelative(true);
        }

        return m_simulatedSwerve.getMeasuredSpeedsRobotRelative(true);
    }

    @Override
    public void simulationPeriodic() {
        m_simulatedSwerve.periodic();

        updateOdometry();

        field2d.setRobotPose(m_simulatedSwerve.getActualPoseInSimulationWorld());
        odometry_field2d.setRobotPose(getPose());

        publisher.set(new SwerveModuleState[] {
            m_simulatedSwerve.getMeasuredStates()[0],
            m_simulatedSwerve.getMeasuredStates()[1],
            m_simulatedSwerve.getMeasuredStates()[2],
            m_simulatedSwerve.getMeasuredStates()[3],
        });
    }
}