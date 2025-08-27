package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class AlignCommand extends Command {
    SwerveSubsystem m_swerveSubsystem;
    VisionSubsystem m_visionSubsystem;

    public AlignCommand(SwerveSubsystem swerve, VisionSubsystem vision) {
        this.m_swerveSubsystem = swerve;
        this.m_visionSubsystem = vision;
    }

    @Override
    public void execute() {
        double rot = m_visionSubsystem.getDesiredRotSpeed();
        this.m_swerveSubsystem.drive(
            0,
            0,
            rot,
            false);
    }
}
