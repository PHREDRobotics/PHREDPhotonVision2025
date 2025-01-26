package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModule {
    public SwerveModuleState getState();
    public SwerveModulePosition getPosition();
    public double getDriveTemp();
    public double getTurnTemp();
    public void setDesiredState(SwerveModuleState desiredState);
}
