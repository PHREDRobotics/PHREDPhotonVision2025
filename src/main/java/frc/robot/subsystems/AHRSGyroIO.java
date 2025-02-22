package frc.robot.subsystems;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * CImplements the NavX side of the GyroIO interface
 */
public class AHRSGyroIO implements GyroIO {
    private final AHRS m_gyro;

    /**
     * Creates a new AHRSGyroIO.
     * @param comType The communication type for the NavX
     */
    public AHRSGyroIO(NavXComType comType) {
        m_gyro = new AHRS(comType);
    }

    
    /** 
     * @return Rotation2d
     */
    @Override
    public Rotation2d getRotation2d() {
        return m_gyro.getRotation2d();
    }

    @Override
    public double getRate() {
        return m_gyro.getRate();
    }

    @Override
    public void reset() {
        m_gyro.reset();
    }
    
}
