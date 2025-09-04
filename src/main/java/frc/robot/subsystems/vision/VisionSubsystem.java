package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.proto.Photon;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class VisionSubsystem extends SubsystemBase {
  private PhotonCamera camera;

  private Transform3d robotToCamera;
  private Transform3d robotToTarget;

  private ProfiledPIDController pidX;
  private ProfiledPIDController pidY;
  private ProfiledPIDController pidRot;
  private ProfiledPIDController pidAlign;

  private PhotonPipelineResult result = new PhotonPipelineResult();

  /**
   * Creates a vision subsystem
   */
  public VisionSubsystem() {
    pidX = new ProfiledPIDController(
        VisionConstants.kXYP, VisionConstants.kXYI, VisionConstants.kXYD,
        Constants.VisionConstants.kXYControllerConstraints);
    pidY = new ProfiledPIDController(
        VisionConstants.kXYP, VisionConstants.kXYI, VisionConstants.kXYD,
        Constants.VisionConstants.kXYControllerConstraints);
    pidRot = new ProfiledPIDController(
        VisionConstants.kRP, VisionConstants.kRI, VisionConstants.kRD,
        Constants.VisionConstants.kRControllerConstraints);
    pidRot.enableContinuousInput(-Math.PI, Math.PI);
    pidAlign = new ProfiledPIDController(
        VisionConstants.kAP, VisionConstants.kAI, VisionConstants.kAD,
        Constants.VisionConstants.kAControllerConstraints);

    camera = new PhotonCamera(VisionConstants.kCameraName);

    robotToCamera = VisionConstants.robotToCamera1;
  }

  /**
   * Gets the desired chassis speeds to get to the tag, with a {@link Pose2d}
   * offset
   * 
   * @param currentPose the current robot pose
   * @param offset      an offset to the desired position in meters (we don't want
   *                    to run
   *                    into the tag)
   * @return
   */
  public ChassisSpeeds getDesiredAlignSpeeds(Pose2d currentPose, Pose2d offset) {
    if (result.hasTargets()) {
      double aOutput = pidAlign.calculate(currentPose.getY(), robotToTarget.getY() + offset.getY());
      ChassisSpeeds speeds = new ChassisSpeeds(0, 0, aOutput);

      SmartDashboard.putString("desiredAlignSpeeds", speeds.toString());
      return speeds;

    }

    return new ChassisSpeeds();
  }

  public ChassisSpeeds getDesiredSpeeds(Pose2d currentPose, Pose2d offset) {
    if (result.hasTargets()) {
      double xOutput = pidX.calculate(currentPose.getX(), robotToTarget.getX() + offset.getX());
      double yOutput = pidY.calculate(currentPose.getY(), robotToTarget.getY() + offset.getY());
      double rotOutput = -pidRot.calculate(currentPose.getRotation().getRadians(), robotToTarget.getRotation().getAngle() + offset.getRotation().getRadians());
      
      ChassisSpeeds speeds = new ChassisSpeeds(xOutput, yOutput+rotOutput*VisionConstants.kStrafeMult, rotOutput);

      SmartDashboard.putString("desiredSpeeds", speeds.toString());
      return speeds;

    }

    return new ChassisSpeeds();
  }
public boolean hasValidTarget(){
  if(result.hasTargets()){
    return true;
  } else {
    return false;
  }
}
  /**
   * Gets the estimated pose of the robot relative to the field
   * 
   * @return The estimated robot pose
   */
  public Optional<Pose2d> getEstimatedGlobalPose() {
    PhotonTrackedTarget target = result.getBestTarget();

    int id = target.getFiducialId();
    var tagPoseOpt = Constants.VisionConstants.kAprilTagLayout.getTagPose(id);

    Pose3d tagPose = tagPoseOpt.get();
    Transform3d cameraToTag = target.getBestCameraToTarget();
    Transform3d tagToCamera = cameraToTag.inverse();
    Pose3d cameraPose = tagPose.transformBy(tagToCamera);
    Pose3d robotPose = cameraPose.transformBy(robotToCamera.inverse());

    return Optional.ofNullable(robotPose.toPose2d());
  }
  public Optional<Pose2d> getEstimatedRelativePose() {
    PhotonTrackedTarget target = result.getBestTarget();
    Transform3d cameraToTag = target.getBestCameraToTarget();
    Transform3d tagToCamera = cameraToTag.inverse();
    Pose3d cameraPose = new Pose3d(tagToCamera.getX(), tagToCamera.getY(), tagToCamera.getZ(), tagToCamera.getRotation());
    
    Pose3d robotPose = cameraPose.transformBy(robotToCamera.inverse());
    SmartDashboard.putString("currentRelativePose", robotPose.toString());
    return Optional.ofNullable(robotPose.toPose2d());
  }
  @Override
  public void periodic() {
    var results = camera.getAllUnreadResults();
    if (!results.isEmpty()) {
      result = results.get(results.size() - 1);

      if (result.hasTargets()) {
        robotToTarget = robotToCamera.plus(result.getBestTarget().getBestCameraToTarget());
        SmartDashboard.putNumber("robotToTarget/X", robotToTarget.getX());
        SmartDashboard.putNumber("robotToTarget/Y", robotToTarget.getY());
        SmartDashboard.putNumber("robotToTarget/Z", robotToTarget.getZ());
        SmartDashboard.putNumber("robotToTarget/Rot", robotToTarget.getRotation().toRotation2d().getRadians());

        SmartDashboard.putString("Estimated pose", getEstimatedGlobalPose().get().toString());
      }
    }
  }
}
