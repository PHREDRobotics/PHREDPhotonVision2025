package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Simulation implementation of {@link SwerveDrive} using maple-sim
 */
public class MapleSimSwerve extends SubsystemBase implements SwerveDrive {
    private final SelfControlledSwerveDriveSimulation m_simulatedSwerve;
    private final Field2d field2d;

    private final StructArrayPublisher<SwerveModuleState> publisher;

    public MapleSimSwerve() {
        final DriveTrainSimulationConfig config = new DriveTrainSimulationConfig(
            Pounds.of(Constants.PhysicalConstants.kRobotMassPounds),
            Inches.of(Constants.PhysicalConstants.kBumperLength),
            Inches.of(Constants.PhysicalConstants.kBumperLength),
            Inches.of(Constants.PhysicalConstants.kFrontLeftLocationInches.getX() * 2),
            Inches.of(Constants.PhysicalConstants.kFrontLeftLocationInches.getY() * 2),
            () -> COTS.ofMark4i(DCMotor.getNEO(1), DCMotor.getNEO(1), 1, 1).get(),
            () -> COTS.ofNav2X().get());
        
        this.m_simulatedSwerve = new SelfControlledSwerveDriveSimulation(
            new SwerveDriveSimulation(config, new Pose2d(8, 3, new Rotation2d())));
        
        SimulatedArena.getInstance().addDriveTrainSimulation(m_simulatedSwerve.getDriveTrainSimulation());

        field2d = new Field2d();
        SmartDashboard.putData("Sim field", field2d);
        publisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("/SwerveStates", SwerveModuleState.struct).publish();
    }

    @Override
    public void drive(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rot, BooleanSupplier fieldOriented) {
        this.m_simulatedSwerve.runChassisSpeeds(
            new ChassisSpeeds(xSpeed.getAsDouble(), ySpeed.getAsDouble(), rot.getAsDouble()),
            new Translation2d(),
            fieldOriented.getAsBoolean(),
            true);
    }

    @Override
    public void simulationPeriodic() {
        m_simulatedSwerve.periodic();

        field2d.setRobotPose(m_simulatedSwerve.getActualPoseInSimulationWorld());

        publisher.set(new SwerveModuleState[] {
            m_simulatedSwerve.getMeasuredStates()[0],
            m_simulatedSwerve.getMeasuredStates()[1],
            m_simulatedSwerve.getMeasuredStates()[2],
            m_simulatedSwerve.getMeasuredStates()[3],
        });
    }
}
