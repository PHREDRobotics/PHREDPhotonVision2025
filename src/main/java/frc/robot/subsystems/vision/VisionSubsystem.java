package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PhotonVisionConstants;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {
  private PhotonCamera camera;
  private PhotonPipelineResult result;
  private PhotonPoseEstimator photonPoseEstimator;

  private Transform3d robotToCamera;
  private Transform3d robotToTarget;
  private Pose3d prevRobotPose;

  private ProfiledPIDController pidX;
  private ProfiledPIDController pidY;
  private ProfiledPIDController pidRot;

  /**
   * Creates a vision subsystem
   */
  public VisionSubsystem() {
    pidX = new ProfiledPIDController(
        PhotonVisionConstants.kXYP, PhotonVisionConstants.kXYI, PhotonVisionConstants.kXYD,
        Constants.PhotonVisionConstants.kXYControllerConstraints);
    pidY = new ProfiledPIDController(
        PhotonVisionConstants.kXYP, PhotonVisionConstants.kXYI, PhotonVisionConstants.kXYD,
        Constants.PhotonVisionConstants.kXYControllerConstraints);
    pidRot = new ProfiledPIDController(
        PhotonVisionConstants.kRP, PhotonVisionConstants.kRI, PhotonVisionConstants.kRD,
        Constants.PhotonVisionConstants.kRControllerConstraints);

    camera = new PhotonCamera(VisionConstants.kCameraName);

    robotToCamera = PhotonVisionConstants.robotToCamera1;

    photonPoseEstimator = new PhotonPoseEstimator(
        Constants.PhotonVisionConstants.aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
        robotToCamera);
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
   * @param prevEstimatedRobotPose The last estimated pose
   * @param result                 The new result
   * @return The estimated robot pose
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    photonPoseEstimator.setReferencePose(prevRobotPose);
    return photonPoseEstimator.update(result);
  }

  @Override
  public void periodic() {
    result = camera.getAllUnreadResults().get(0);

    if (result.hasTargets()) {
      robotToTarget = robotToCamera.plus(result.getBestTarget().getBestCameraToTarget());
    }

    if (getEstimatedGlobalPose().isPresent()) {
      prevRobotPose = getEstimatedGlobalPose().get().estimatedPose;
    }
  }
}
