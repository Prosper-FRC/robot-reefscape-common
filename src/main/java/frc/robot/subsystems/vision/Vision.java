package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import frc.robot.utils.debugging.LoggedTunableNumber;
import static frc.robot.subsystems.vision.VisionConstants.kSingleStdDevs;
import static frc.robot.subsystems.vision.VisionConstants.kMultiStdDevs;
import static frc.robot.subsystems.vision.VisionConstants.KUseSingleTagTransform;
import static frc.robot.subsystems.vision.VisionConstants.kAmbiguityThreshold;;

public class Vision {
    private VisionIO[] cameras;
    private VisionIOInputsAutoLogged[] camerasData;

    private static final LoggedTunableNumber kSingleXYStdev = new LoggedTunableNumber(
        "Vision/kSingleXYStdev", kSingleStdDevs.get(0));
    private static final LoggedTunableNumber kMultiXYStdev = new LoggedTunableNumber(
        "Vision/kMultiXYStdev", kMultiStdDevs.get(0));

    private final AprilTagFieldLayout k2025Field = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    public Vision(VisionIO[] cameras) {
        Logger.recordOutput("Vision/UseSingleTagTransform", KUseSingleTagTransform);
        this.cameras = cameras;
        camerasData = new VisionIOInputsAutoLogged[cameras.length];
        for(int i = 0; i < cameras.length; i++) {
            camerasData[i] = new VisionIOInputsAutoLogged();
        }
    }

    public void periodic(Pose2d lastRobotPose, Pose2d simOdomPose) {
        for(int i = 0; i < cameras.length; i++) {
            cameras[i].updateInputs(camerasData[i], lastRobotPose, simOdomPose);
            Logger.processInputs("Vision/"+camerasData[i].camName, camerasData[i]);
            Logger.recordOutput("Vision/"+camerasData[i].camName+"/Pose", camerasData[i].latestEstimatedRobotPose.toPose2d());
            Logger.recordOutput("Vision/"+camerasData[i].camName+"/X", camerasData[i].latestEstimatedRobotPose.getRotation().getX());
            Logger.recordOutput("Vision/"+camerasData[i].camName+"/Y", camerasData[i].latestEstimatedRobotPose.getRotation().getY());
            Logger.recordOutput("Vision/"+camerasData[i].camName+"/Z", camerasData[i].latestEstimatedRobotPose.getRotation().getZ());
        }

    }

    // Gets the vision data. Standard Deviations are how much we trus the vision value
    public VisionObservation[] getVisionObservations() {
        VisionObservation[] observations = new VisionObservation[cameras.length];
        int i = 0;
        // STANDARD DEVIATION CALCULATIONS \\
        for(VisionIOInputsAutoLogged camData : camerasData) {
            // No point in adding vision data if it doesn't exist
            if(camData.hasTarget && camData.hasBeenUpdated) {
                // Average distance from tag, and the number of tags to determine estimate stability
                double numberOfTargets = camData.numberOfTargets;
                double avgDistMeters = 0.0;
                for(int r = 0; r < camData.latestTagTransforms.length; r++) {
                    if(camData.latestTagTransforms[r] != null) {
                        if(camData.latestTagAmbiguities[r] < kAmbiguityThreshold) {
                            avgDistMeters += camData.latestTagTransforms[r].getTranslation().getNorm();
                        } else {
                            numberOfTargets -= 1;
                        }
                    }
                }

                // No point in adding vision data if it doesn't exist
                if(numberOfTargets == 0) {
                    observations[i] = new VisionObservation(
                        true, 
                        camData.latestEstimatedRobotPose.toPose2d(), 
                        VecBuilder.fill(
                            Double.MAX_VALUE, 
                            Double.MAX_VALUE, 
                            Double.MAX_VALUE), 
                        camData.latestTimestamp, camData.camName);
                }

                avgDistMeters /= numberOfTargets;
                Logger.recordOutput("Vision/AvgDistMeters", avgDistMeters);

                double xyScalar = Math.pow(avgDistMeters, 2) / (numberOfTargets);

                // Cases where we shouldn't add vision measurements
                if(numberOfTargets == 0 || (numberOfTargets == 1 && avgDistMeters > 3.5)) {
                    observations[i] = new VisionObservation(
                        true,
                        camData.latestEstimatedRobotPose.toPose2d(), 
                        VecBuilder.fill(
                            Double.MAX_VALUE, 
                            Double.MAX_VALUE, 
                            Double.MAX_VALUE), 
                        camData.latestTimestamp, camData.camName);
                // In other cases, run single-tag calibration
                } else if(numberOfTargets == 1) {
                    Pose2d singleTagPose = new Pose2d();
                    if(KUseSingleTagTransform) {
                        singleTagPose = 
                            // Pose of involved tag
                            k2025Field.getTagPose(camData.aprilTagID).get().toPose2d()
                            // Transform pose to camera
                            .plus(new Transform2d(
                                    camData.cameraToApriltag.getX(), camData.cameraToApriltag.getY(), 
                                    camData.cameraToApriltag.getRotation().toRotation2d()))
                            // Transform camera to robot center
                            .plus(toTransform2d(camData.cameraToRobot.inverse()));
                    } else {
                        singleTagPose = camData.latestEstimatedRobotPose.toPose2d();
                    }
                        observations[i] = new VisionObservation(
                            true,
                            singleTagPose, 
                            VecBuilder.fill(
                                kSingleXYStdev.get() * xyScalar, 
                                kSingleXYStdev.get() * xyScalar, 
                                Double.MAX_VALUE), 
                            camData.latestTimestamp, camData.camName);
                // In other cases, run multi-tag calibration
                } else {
                    observations[i] = new VisionObservation(
                        true,
                        camData.latestEstimatedRobotPose.toPose2d(), 
                        VecBuilder.fill(
                            kMultiXYStdev.get() * xyScalar, 
                            kMultiXYStdev.get() * xyScalar, 
                            Double.MAX_VALUE), 
                        camData.latestTimestamp, camData.camName);
                }
            } else {
                observations[i] = new VisionObservation(
                    false, 
                    new Pose2d(), 
                    VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE), 
                    camData.latestTimestamp, camData.camName);
            }
            i++;
       }
        return observations;
    }

    private Transform2d toTransform2d(Transform3d transform) {
        return new Transform2d(transform.getX(), transform.getY(), transform.getRotation().toRotation2d());
    }

    public record VisionObservation(boolean hasObserved, Pose2d pose, Vector<N3> stdDevs, double timeStamp, String camName) {}
}