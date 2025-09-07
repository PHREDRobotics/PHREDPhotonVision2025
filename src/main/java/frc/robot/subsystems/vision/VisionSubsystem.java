package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {
  private PhotonCamera camera;

  private Transform3d robotToCamera;
  private Transform3d robotToTarget;

  private PhotonPipelineResult result = new PhotonPipelineResult();

  /**
   * Creates a vision subsystem
   */
  public VisionSubsystem() {
    camera = new PhotonCamera(VisionConstants.kCameraName);

    robotToCamera = VisionConstants.robotToCamera1;
  }

  public Pose2d getTargetPose(Pose2d tag) {
    Pose2d targetPose = new Pose2d(
        tag.getX()
            + VisionConstants.kMetersFromAprilTag * Math.cos(tag.getRotation().getRadians()),
        tag.getY()
            + VisionConstants.kMetersFromAprilTag * Math.sin(tag.getRotation().getRadians()),
        tag.getRotation().rotateBy(new Rotation2d(Math.PI)));

    return targetPose;
  }

  public boolean hasValidTarget() {
    if (result.hasTargets()) {
      return true;
    } else {
      return false;
    }
  }

  public Transform3d getTagPose() {
    if (!result.hasTargets()) {
      return null;
    }
    Transform3d tagTransform = result.getBestTarget().getBestCameraToTarget();
    return tagTransform;
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
    Pose3d cameraPose = new Pose3d(tagToCamera.getX(), tagToCamera.getY(), tagToCamera.getZ(),
        tagToCamera.getRotation());

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
