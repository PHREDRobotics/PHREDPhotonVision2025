package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class TargetVeryLargePuppyDog extends Command {
    



  private SwerveSubsystem m_swerveSubsystem;
  private VisionSubsystem m_visionSubsystem;

 private ChassisSpeeds speeds;

  public TargetVeryLargePuppyDog(SwerveSubsystem swerveSubsystem, VisionSubsystem visionSubsystem) {
    this.m_swerveSubsystem = swerveSubsystem;
    this.m_visionSubsystem = visionSubsystem;
    addRequirements(swerveSubsystem);
    addRequirements(visionSubsystem);
  }

  @Override
  public void execute() {
    Pose2d currentPose;
    Transform3d aprilTransform;
    if (!m_visionSubsystem.hasValidTarget()) {
        currentPose = VisionConstants.kOffset;
        aprilTransform = new Transform3d();

    } else {
        currentPose = m_visionSubsystem.getEstimatedRelativePose().get();
        aprilTransform = m_visionSubsystem.getTagPose();

    }
    //Step 1: Get tag transform
    //Step 2: Get target position off of tag transform
    //Pose2d targetPose = m_visionSubsystem.getTargetPose(new Pose2d(aprilTransform.getX(), aprilTransform.getY(), aprilTransform.getRotation().toRotation2d()));
    //Step 3: Get current pose off of tag, passing target pose if null

    //Step 4: Set speeds using PIDs with current pose and target pose
    
    speeds = m_visionSubsystem.transformPosition(currentPose, VisionConstants.kOffset);
    //Step 5: Drive the robot
    m_swerveSubsystem.drive(speeds,false);

    SmartDashboard.putString("Step 1", aprilTransform.toString());
    //SmartDashboard.putString("Step 2", targetPose.toString());
    SmartDashboard.putString("Step 3", currentPose.toString());
    SmartDashboard.putString("Step 4", speeds.toString());
  }
}
