package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class GoToTagCommand extends Command {

    private SwerveSubsystem m_swerveSubsystem;

    public GoToTagCommand(SwerveSubsystem swerveSubsystem, VisionSubsystem visionSubsystem) {
        this.m_swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);
    }

    public void initialize() {
        m_swerveSubsystem.resetPIDs(VisionConstants.kOffset);
    }

    @Override
    public void execute() {
        m_swerveSubsystem.driveTo(VisionConstants.kOffset);
    }
}