package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {
  private PhotonCamera camera;
  private PhotonPipelineResult result;

  private Transform3d robotToCamera;
  private Transform3d robotToTarget;

  private ProfiledPIDController pidX;
  private ProfiledPIDController pidY;
  private ProfiledPIDController pidRot;

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
   * Calculates the desired x speed
   * 
   * @return The desired x speed
   */
  public double getDesiredXspeed() {
    if (result.hasTargets()) {
      return pidX.calculate(0, robotToTarget.getX());
    } else {
      return 0;
    }
  }

  /**
   * Calculates the desired y speed
   * 
   * @return The desired y speed
   */
  public double getDesiredYspeed() {
    if (result.hasTargets()) {
      return pidY.calculate(0, robotToTarget.getY());
    } else {
      return 0;
    }
  }

  /**
   * Calculates the desired rotation speed
   * 
   * @return The desired rotation speed
   */
  public double getDesiredRotSpeed() {
    if (result.hasTargets()) {
      return pidRot.calculate(0, robotToTarget.getRotation().getAngle());
    } else {
      return 0;
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

  @Override
  public void periodic() {
    result = camera.getAllUnreadResults().get(0);

    if (result.hasTargets()) {
      robotToTarget = robotToCamera.plus(result.getBestTarget().getBestCameraToTarget());
    }
  }
}
