package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

import frc.robot.Constants.VisionConstants;

public class PhotonVisionSubsystem {
  PhotonCamera camera;
  PhotonPipelineResult result;
  PhotonTrackedTarget bestTarget;
  Transform3d cameraToTarget;
  ProfiledPIDController pidX;
  ProfiledPIDController pidY;
  ProfiledPIDController pidZ;
  ProfiledPIDController pidYaw;

  private PhotonVisionSubsystem() {
    camera = new PhotonCamera(VisionConstants.kCameraName);

  }

  public void update() {
    result = camera.getLatestResult();
    bestTarget = result.getBestTarget();
    cameraToTarget = bestTarget.getBestCameraToTarget();
  }

  public Pose3d getRobotToTarget() {
    Pose3d robotToTarget = new Pose3d().plus(VisionConstants.kRobotToCamera1.plus(cameraToTarget));
    return robotToTarget;
  }
}
