package frc.robot.subsystems;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;

public class AHRSGyroIO implements GyroIO {
    private final AHRS m_gyro;

    public AHRSGyroIO(NavXComType comType) {
        m_gyro = new AHRS(comType);
    }

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
