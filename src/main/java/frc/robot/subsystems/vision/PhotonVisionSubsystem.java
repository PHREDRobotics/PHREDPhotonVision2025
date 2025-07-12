package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.proto.Photon;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.PhotonVisionConstants;
import frc.robot.Constants.VisionConstants;

public class PhotonVisionSubsystem {

    PhotonCamera camera;
    PhotonPipelineResult result;
    PhotonTrackedTarget bestTarget;
    PhotonPoseEstimator photonPoseEstimator;

    Transform3d robotToCamera;
    Transform3d cameraToTarget;
    Transform3d robotToTarget;
    Pose3d prevRobotPose;
    Pose3d currentRobotPose;
    public Optional<EstimatedRobotPose> currentRobotEstimation;

    public ProfiledPIDController pidX;
    public ProfiledPIDController pidY;
    public ProfiledPIDController pidRot;

    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // IDK WHATS GOING ON WHY ARE THERE SO MANY TRANSFORM3D, cuz there has to be
    // Also this code is prob bad atm, but I think its needed somewhere
    public PhotonVisionSubsystem() {
        // Initialize PIDs
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
        // camera1
        robotToCamera = PhotonVisionConstants.robotToCamera1;

        // Construct PhotonPoseEstimator
        photonPoseEstimator = new PhotonPoseEstimator(
                Constants.PhotonVisionConstants.aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
                robotToCamera);
    }

    // Calculates the xSpeed based off of the targets position relative to the robot
    public double getXspeed() {
        if (result.hasTargets()) {
            return pidX.calculate(0, robotToTarget.getX());
        } else {
            return 0;
        }
    }

    // Calculates the ySpeed based off of the targets position relative to the robot
    public double getYspeed() {
        if (result.hasTargets()) {
            return pidY.calculate(0, robotToTarget.getY());
        } else {
            return 0;
        }
    }

    // Calculates the rotation speed based off of the targets rotation relative to
    // the robot
    public double getRotSpeed() {
        if (result.hasTargets()) {
            return pidRot.calculate(0, robotToTarget.getRotation().getAngle());
        } else {
            return 0;
        }
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose3d prevEstimatedRobotPose,
            PhotonPipelineResult result) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update(result);
    }

    public void update() {
        // Sets the prevPose to the currentPose
        prevRobotPose = currentRobotPose;
        result = camera.getLatestResult();
        // Checks if the camera has a target, then does target related operations
        if (result.hasTargets() == true) {
            bestTarget = result.getBestTarget();
            cameraToTarget = bestTarget.getBestCameraToTarget();
            robotToTarget = robotToCamera.plus(cameraToTarget);
        }
        // Sets the currentPose using the prevPose
        // gives global pos, ignore for now
        currentRobotEstimation = getEstimatedGlobalPose(prevRobotPose, result);
        currentRobotPose = currentRobotEstimation.orElse(null).estimatedPose;
    }
}
