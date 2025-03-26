package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * Subsystem for controlling swerve simulation
 */
public class SwerveSimulation {
  private final SelfControlledSwerveDriveSimulation m_swerveSimulation;
  private final Field2d m_field;

  private final StructArrayPublisher<SwerveModuleState> publisher;

  public SwerveSimulation(GyroIO gyroSim) {
    publisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("/SwerveStates", SwerveModuleState.struct).publish();

    SwerveModuleSimulationConfig moduleConfig = new SwerveModuleSimulationConfig(
        DCMotor.getNEO(1),
        DCMotor.getNEO(1),
        Constants.SwerveConstants.kDrivingMotorReduction,
        Constants.SwerveConstants.kTurningMotorReduction,
        Constants.SimConstants.kDrivingFrictionVolts,
        Constants.SimConstants.kTurningFrictionVolts,
        Meters.of(Constants.SwerveConstants.kWheelRadius),
        KilogramSquareMeters.of(0.02),
        1.2);

    @SuppressWarnings("unchecked")
    DriveTrainSimulationConfig driveTrainConfig = new DriveTrainSimulationConfig(
        Pounds.of(Constants.PhysicalConstants.kRobotMassPounds),
        Inches.of(Constants.PhysicalConstants.kBumperLength),
        Inches.of(Constants.PhysicalConstants.kBumperLength),
        Inches.of(Constants.PhysicalConstants.kTrackLength),
        Inches.of(Constants.PhysicalConstants.kTrackLength),
        () -> new GyroSimulation(0, 0),
        () -> new SwerveModuleSimulation(moduleConfig));

    m_swerveSimulation = new SelfControlledSwerveDriveSimulation(
        new SwerveDriveSimulation(
            driveTrainConfig,
            new Pose2d(8.1, 7, new Rotation2d())));

    m_field = new Field2d();
    SmartDashboard.putData("Sim Robot", m_field);

    SimulatedArena.getInstance().addDriveTrainSimulation(m_swerveSimulation.getDriveTrainSimulation());
  }

  public Pose2d getPose() {
    return m_swerveSimulation.getActualPoseInSimulationWorld();
  }

  /**
   * @param xSpeed
   * @param ySpeed
   * @param rot
   * @param fieldOriented
   * @param gyroRotation
   */
  public void update(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rot, BooleanSupplier fieldOriented,
      Rotation2d gyroRotation) {
    // m_gyro.setRotation(gyroRotation);

    ChassisSpeeds speeds = ChassisSpeeds.discretize(
        xSpeed.getAsDouble() * Constants.PhysicalConstants.kMaxSpeed,
        ySpeed.getAsDouble() * Constants.PhysicalConstants.kMaxSpeed,
        -rot.getAsDouble() * Constants.PhysicalConstants.kMaxAngularSpeed,
        Constants.SwerveConstants.kDtSeconds);

    m_swerveSimulation.runChassisSpeeds(speeds, new Translation2d(), fieldOriented.getAsBoolean(), true);
    m_swerveSimulation.periodic();
    m_field.setRobotPose(m_swerveSimulation.getActualPoseInSimulationWorld());

    SmartDashboard.putString("moduleStatessim", speeds.toString());
    SmartDashboard.putString("Speeds in sim", m_swerveSimulation.getActualSpeedsRobotRelative().toString());
    SmartDashboard.putNumber("Sim Gyro", m_swerveSimulation.getOdometryEstimatedPose().getRotation().getDegrees());

    publisher.set(m_swerveSimulation.getMeasuredStates());
  }

  /*
   * public void update(DoubleSupplier xSpeed, DoubleSupplier ySpeed,
   * DoubleSupplier rot, BooleanSupplier fieldOriented,
   * Rotation2d gyroRotation) {
   * 
   * m_gyro.setRotation(gyroRotation);
   * 
   * SwerveModuleState[] swerveStates =
   * Constants.SwerveConstants.kKinematics.toSwerveModuleStates(ChassisSpeeds.
   * discretize(
   * fieldOriented.getAsBoolean()
   * ? ChassisSpeeds.fromFieldRelativeSpeeds(
   * xSpeed.getAsDouble() * Constants.PhysicalConstants.kMaxSpeed,
   * ySpeed.getAsDouble() * Constants.PhysicalConstants.kMaxSpeed,
   * -rot.getAsDouble() * Constants.PhysicalConstants.kMaxAngularSpeed,
   * gyroRotation)
   * : new ChassisSpeeds(
   * xSpeed.getAsDouble() * Constants.PhysicalConstants.kMaxSpeed,
   * ySpeed.getAsDouble() * Constants.PhysicalConstants.kMaxSpeed,
   * -rot.getAsDouble() * Constants.PhysicalConstants.kMaxAngularSpeed),
   * Constants.SwerveConstants.kDtSeconds));
   * m_swerveSimulation.runSwerveStates(
   * swerveStates);
   * m_swerveSimulation.periodic();
   * m_field.setRobotPose(m_swerveSimulation.getActualPoseInSimulationWorld());
   * 
   * SmartDashboard.putString("moduleStatessim", swerveStates.toString());
   * SmartDashboard.putString("poseinsim",
   * m_swerveSimulation.getActualPoseInSimulationWorld().toString());
   * SmartDashboard.putString("simspeeds",
   * m_swerveSimulation.getActualSpeedsFieldRelative().toString());
   * SmartDashboard.putString("Latest module positions",
   * m_swerveSimulation.getLatestModulePositions().toString());
   * 
   * publisher.set(new SwerveModuleState[] {
   * m_swerveSimulation.getMeasuredStates()[0],
   * m_swerveSimulation.getMeasuredStates()[1],
   * m_swerveSimulation.getMeasuredStates()[2],
   * m_swerveSimulation.getMeasuredStates()[3],
   * });
   * }
   */
}
