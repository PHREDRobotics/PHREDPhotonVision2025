package frc.robot.subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import org.ironmaple.simulation.drivesims.GyroSimulation;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Implements the simulation side of the GyroIO interface
 */
public class SimGyroIO implements GyroIO {
    private final GyroSimulation m_gyro;

    public SimGyroIO() {
        m_gyro = new GyroSimulation(2, 2);
    }

    
    /** 
     * @return Rotation2d
     */
    @Override
    public Rotation2d getRotation2d() {
        return m_gyro.getGyroReading();
    }

    @Override
    public double getRate() {
        return m_gyro.getMeasuredAngularVelocity().in(DegreesPerSecond);
    }

    @Override
    public void reset() {
        m_gyro.setRotation(new Rotation2d());
    }
    
}
