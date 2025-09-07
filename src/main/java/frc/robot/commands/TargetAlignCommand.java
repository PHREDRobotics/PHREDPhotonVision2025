package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class TargetAlignCommand extends Command {
  SwerveSubsystem m_swerveSubsystem;
  VisionSubsystem m_visionSubsystem;

  public TargetAlignCommand(SwerveSubsystem swerve, VisionSubsystem vision) {
    this.m_swerveSubsystem = swerve;
    this.m_visionSubsystem = vision;
  }

  @Override
  public void execute() {
    Pose2d currentPose = this.m_visionSubsystem.getEstimatedRelativePose().get();
    this.m_swerveSubsystem.driveRelativeTo(currentPose,
        new Pose2d(currentPose.getX(), currentPose.getY(), new Rotation2d(Math.PI)));
  }
}
