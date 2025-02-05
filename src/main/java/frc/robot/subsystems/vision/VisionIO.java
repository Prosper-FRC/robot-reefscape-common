package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
    @AutoLog
    public static class VisionIOInputs {
        public String camName = "";
        public boolean isConnected = false;
        public double yaw = 0.0;
        public double pitch = 0.0;
        public double area = 0.0;
        public double latencySeconds = 0.0;
        public boolean hasTarget = false;
        public int numberOfTargets = 0;

        public Transform3d cameraToRobot = new Transform3d();
        public Transform3d cameraToApriltag = new Transform3d();
        public double poseAmbiguity = 0.0;
        public int aprilTagID = 0;
        public Transform3d robotToApriltag = new Transform3d();
        public double latestTimestamp = 0.0;
        public boolean hasBeenUpdated = false;
        public Pose3d latestEstimatedRobotPose = new Pose3d();
        public Transform3d[] latestTagTransforms = new Transform3d[] {
            new Transform3d(), new Transform3d(), new Transform3d(), new Transform3d(), 
            new Transform3d(), new Transform3d(), new Transform3d(), new Transform3d(), 
            new Transform3d(), new Transform3d(), new Transform3d(), new Transform3d(), 
            new Transform3d(), new Transform3d()};
        public double[] latestTagAmbiguities = new double[] {
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        };
    }

    public default void updateInputs(VisionIOInputs inputs, Pose2d lastRobotPose, Pose2d simOdomPose) {}
}