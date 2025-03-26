package frc.robot.subsystems.swerve;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Interface for switching between a real swerve module and a simulated swerve
 * module
 */
public interface SwerveModule {
  public SwerveModuleState getState();

  public SwerveModulePosition getPosition();

  public double getDriveTemp();

  public double getTurnTemp();

  public void setDesiredState(SwerveModuleState desiredState);
}
