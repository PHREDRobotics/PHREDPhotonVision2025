package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.struct.Pose2dStruct;
import edu.wpi.first.wpilibj.DataLogManager;
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

  public void initialize() {
    m_swerveSubsystem.resetPIDs(VisionConstants.kOffset);
  }

  @Override
  public void execute() {
    // Step 1: Get tag transform
    // Step 2: Get target position off of tag transform
    // Step 3: Get current pose off of tag, passing target pose if null
    // Step 4: Set speeds using PIDs with current pose and target pose
    // Step 5: Drive the robot
    Pose2d currentPose = VisionConstants.kOffset;
    if (m_visionSubsystem.hasValidTarget()) {
      currentPose = m_visionSubsystem.getEstimatedRelativePose().get();
      m_swerveSubsystem.driveRelativeTo(currentPose, VisionConstants.kOffset);
      //m_swerveSubsystem.driveRelativeTo(new Pose2d(), new Pose2d());
    } else {
      m_swerveSubsystem.driveRelativeTo(VisionConstants.kOffset, VisionConstants.kOffset);
      //m_swerveSubsystem.driveRelativeTo(new Pose2d(), new Pose2d());
    }

    //DataLogManager.log(currentPose.toString());
  }
}
