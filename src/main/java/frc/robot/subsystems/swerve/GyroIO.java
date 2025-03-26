package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Interface for switching between a real gyro and a simulated gyro.
 */
public interface GyroIO {
  public Rotation2d getRotation2d();

  public double getRate();

  public void reset();
}
