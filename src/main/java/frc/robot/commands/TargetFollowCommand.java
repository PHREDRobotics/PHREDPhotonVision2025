package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class TargetFollowCommand extends Command {
  SwerveSubsystem m_swerveSubsystem;
  VisionSubsystem m_visionSubsystem;

  public TargetFollowCommand(SwerveSubsystem swerve, VisionSubsystem vision) {
    this.m_swerveSubsystem = swerve;
    this.m_visionSubsystem = vision;
  }

  @Override
  public void execute() {
    Pose2d offset = new Pose2d(new Translation2d(-1.5, 0), new Rotation2d(Math.PI)); // distance in meters
    ChassisSpeeds speeds = m_visionSubsystem.getDesiredSpeeds(m_swerveSubsystem.getPose(), offset);
    SmartDashboard.putString("desiredSpeeds", speeds.toString());

    this.m_swerveSubsystem.drive(
        speeds,
        false);
  }
}
