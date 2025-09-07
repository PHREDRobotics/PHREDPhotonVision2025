package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class TargetFollowCommand extends Command {
  SwerveSubsystem m_swerveSubsystem;
  VisionSubsystem m_visionSubsystem;

  public TargetFollowCommand(SwerveSubsystem swerve, VisionSubsystem vision) {
    this.m_swerveSubsystem = swerve;
    this.m_visionSubsystem = vision;

    addRequirements(swerve, vision);
  }

  @Override
  public void execute() {
    if (this.m_visionSubsystem.hasValidTarget()) {
      Pose2d currentPose = this.m_visionSubsystem.getEstimatedRelativePose().get();

      this.m_swerveSubsystem.driveRelativeTo(currentPose, new Pose2d(Constants.VisionConstants.kOffset.getX(),
          Constants.VisionConstants.kOffset.getY(), currentPose.getRotation()));
    } else {
      this.m_swerveSubsystem.driveRelativeTo(new Pose2d(), new Pose2d());
    }
  }
}
