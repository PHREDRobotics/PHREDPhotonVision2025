package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroIO {
    public Rotation2d getRotation2d();
    public double getRate();
    public void reset();
}
