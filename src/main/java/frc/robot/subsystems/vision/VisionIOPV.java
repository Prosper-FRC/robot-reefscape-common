package frc.robot.subsystems.vision;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants;
import frc.robot.Constants.Mode;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionIOPV implements VisionIO {
    private String camName;
    private PhotonCamera limelightCam;
    private PhotonPoseEstimator poseEstimator;
    private Transform3d cameraTransform;

    private PhotonCameraSim limelightSim;
    private VisionSystemSim visionSim;

    public VisionIOPV(String name, Transform3d cameraTransform) {
        camName = name;
        limelightCam = new PhotonCamera(camName);
        this.cameraTransform = cameraTransform;
        // Don't worry about it
        PhotonCamera.setVersionCheckEnabled(false);

        poseEstimator = new PhotonPoseEstimator(
            AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField), 
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraTransform);
        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_LAST_POSE);

        if(Constants.kCurrentMode == Mode.SIM) {
            // Create the vision system simulation which handles cameras and targets on the field.
            visionSim = new VisionSystemSim("main");
            // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
            visionSim.addAprilTags(AprilTagFields.k2025Reefscape.loadAprilTagLayoutField());
            // Create simulated camera properties. These can be set to mimic your actual camera.
            var cameraProp = new SimCameraProperties();
            cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(75));
            cameraProp.setCalibError(0.3, 0.20);
            cameraProp.setFPS(60);
            cameraProp.setAvgLatencyMs(5);
            cameraProp.setLatencyStdDevMs(15);
            // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
            // targets.
            limelightSim = new PhotonCameraSim(limelightCam, cameraProp);
            // Add the simulated camera to view the targets on this simulated field.
            visionSim.addCamera(limelightSim, cameraTransform);
        }
    }

    @Override
    public void updateInputs(VisionIOInputs inputs, Pose2d lastRobotPose, Pose2d simOdomPose) {
        inputs.camName = camName;
        inputs.cameraToRobot= cameraTransform;
        // To stop the dangerous case where the camera disconnects, and causes the code to crash
        try {
            // Updates the position from which the cameras have to look at
            if (Constants.kCurrentMode == Mode.SIM) {
                visionSim.update(simOdomPose);
            }
            
            // Gets the camera data
            List<PhotonPipelineResult> unreadResults = limelightCam.getAllUnreadResults();
            poseEstimator.setLastPose(lastRobotPose);
            inputs.hasBeenUpdated = !unreadResults.isEmpty();
            if(!unreadResults.isEmpty()) {
                PhotonPipelineResult result = unreadResults.get(unreadResults.size()-1);
                Optional<EstimatedRobotPose> latestEstimatedRobotPose = poseEstimator.update(result);

                // Adds it to data streaming
                inputs.isConnected = limelightCam.isConnected();

                inputs.hasTarget = result.hasTargets();
                if (result.hasTargets()) {
                    PhotonTrackedTarget target = result.getBestTarget();
                    inputs.cameraToApriltag = target.getBestCameraToTarget();
                    inputs.robotToApriltag = target.getBestCameraToTarget().plus(cameraTransform);
                    inputs.aprilTagID = target.getFiducialId();
                    inputs.poseAmbiguity = target.getPoseAmbiguity();
                    inputs.yaw = target.getYaw();
                    inputs.pitch = target.getPitch();
                    inputs.area = target.getArea();
                    inputs.latencySeconds = result.getTimestampSeconds() / 1000.0;

                    latestEstimatedRobotPose.ifPresent(est -> {
                        inputs.latestEstimatedRobotPose = latestEstimatedRobotPose.get().estimatedPose;

                        ArrayList<Transform3d> tagTs = new ArrayList<>();
                        double[] ambiguities = new double[latestEstimatedRobotPose.get().targetsUsed.size()];
                        if(latestEstimatedRobotPose.get().targetsUsed.size() > 0) {
                            for(int i = 0; i < latestEstimatedRobotPose.get().targetsUsed.size(); i++) {
                                tagTs.add(latestEstimatedRobotPose.get().targetsUsed.get(i).getBestCameraToTarget());
                                ambiguities[i] = latestEstimatedRobotPose.get().targetsUsed.get(i).getPoseAmbiguity();
                            }
                        }
   
                        inputs.numberOfTargets = latestEstimatedRobotPose.get().targetsUsed.size();
                        inputs.latestTagTransforms = tagTs.toArray(Transform3d[]::new);
                        inputs.latestTagAmbiguities = ambiguities;
        
                        inputs.latestTimestamp = result.getTimestampSeconds();
                    });
                }
            }
            // In case camera goes brrr
        } catch(Exception e) {
            e.printStackTrace();
            inputs.isConnected = false;
            inputs.yaw = 0.0;
            inputs.pitch = 0.0;
            inputs.area = 0.0;
            inputs.latencySeconds = 0.0;
            inputs.hasTarget = false;
            inputs.numberOfTargets = 0;
        
            inputs.cameraToApriltag = new Transform3d();
            inputs.poseAmbiguity = 0.0;
            inputs.aprilTagID = 0;
            inputs.robotToApriltag = new Transform3d();
            inputs.latestTimestamp = 0.0;
            inputs.latestEstimatedRobotPose = new Pose3d();
            inputs.latestTagTransforms = new Transform3d[14];
            inputs.latestTagAmbiguities = new double[14];
        }
    }
}