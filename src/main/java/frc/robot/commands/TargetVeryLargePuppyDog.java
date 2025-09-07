package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class TargetVeryLargePuppyDog extends Command {

  private SwerveSubsystem m_swerveSubsystem;
  private VisionSubsystem m_visionSubsystem;

  public TargetVeryLargePuppyDog(SwerveSubsystem swerveSubsystem, VisionSubsystem visionSubsystem) {
    this.m_swerveSubsystem = swerveSubsystem;
    this.m_visionSubsystem = visionSubsystem;
    addRequirements(swerveSubsystem);
    addRequirements(visionSubsystem);
  }

  @Override
  public void execute() {
    // Step 1: Get tag transform
    // Step 2: Get target position off of tag transform
    // Step 3: Get current pose off of tag, passing target pose if null
    // Step 4: Set speeds using PIDs with current pose and target pose
    // Step 5: Drive the robot

    Pose2d currentPose;
    if (m_visionSubsystem.hasValidTarget()) {
      currentPose = m_visionSubsystem.getEstimatedRelativePose().get();
      m_swerveSubsystem.driveRelativeTo(currentPose, VisionConstants.kOffset);
    } else {
      m_swerveSubsystem.driveRelativeTo(new Pose2d(), new Pose2d());
    }
  }
}
