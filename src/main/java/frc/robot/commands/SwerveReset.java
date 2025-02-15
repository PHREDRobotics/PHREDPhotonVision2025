package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveReset extends Command{
    private SwerveSubsystem m_swerveSubsystem;

    public SwerveReset(SwerveSubsystem swerveSubsystem) {
        m_swerveSubsystem = swerveSubsystem;
    }

    @Override
    public void initialize() {
        m_swerveSubsystem.resetOdometry(new Pose2d());
    }
}
