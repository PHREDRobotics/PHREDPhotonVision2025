package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {
  private PhotonCamera camera;

  private Transform3d robotToCamera;
  private Transform3d robotToTarget;

  private ProfiledPIDController pidX;
  private ProfiledPIDController pidY;
  private ProfiledPIDController pidRot;

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

    camera = new PhotonCamera(VisionConstants.kCameraName);

    robotToCamera = VisionConstants.robotToCamera1;
  }

  /**
   * Gets the desired chassis speeds to get to the tag, with a {@link Pose2d}
   * offset
   * 
   * @param currentPose the current robot pose
   * @param offset      an offset to the desired position in meters (we don't want to run
   *                    into the tag)
   * @return
   */
  public ChassisSpeeds getDesiredSpeeds(Pose2d currentPose, Pose2d offset) {
    if (result.hasTargets()) {
      return new ChassisSpeeds(pidX.calculate(currentPose.getX(), robotToTarget.getX() + offset.getX()),
          pidY.calculate(currentPose.getY(), robotToTarget.getY() + offset.getY()),
          pidRot.calculate(currentPose.getRotation().getRadians(), robotToTarget.getRotation().getAngle() + offset.getRotation().getRadians()));
    }
    return new ChassisSpeeds();
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

  @Override
  public void periodic() {
    var results = camera.getAllUnreadResults();
    if (!results.isEmpty()) {
      result = results.get(results.size() - 1);

      if (result.hasTargets()) {
        robotToTarget = robotToCamera.plus(result.getBestTarget().getBestCameraToTarget());
      }
    }
  }
}
