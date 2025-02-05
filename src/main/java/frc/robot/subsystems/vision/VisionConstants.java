package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotBase;
// import frc.robot.subsystems.drive.DriveConstants;

public class VisionConstants {
    // From CAD and decided by you in configuration
    public static final String kLeftCamName = "LEFT";
    public static final Transform3d kLeftCamTransform = new Transform3d(
        new Translation3d(0.3, -0.3, 0.19),
        new Rotation3d(1.0, 1.0, 1.0)
    );

    public static final String kRightCamName = "RIGHT";
    public static final Transform3d kRightCamTransform = new Transform3d(
        new Translation3d(-0.3, -0.3, 0.19),
        new Rotation3d(1.0, 1.0, 1.0)
    );

    // TODO: SET TO FALSE UNLESS YOU ACTUALLY KNOW WHAT THIS DOES
    // This turns on a implementation of Rembrandts single tag vision algorithm
    public static final boolean KUseSingleTagTransform = false;

    // Tuned by your
    public static final Vector<N3> kSingleStdDevs = (RobotBase.isReal()) ?
        VecBuilder.fill(0.01, 0.01, 5.0) : VecBuilder.fill(0.01, 0.01, 5.0);
    public static final Vector<N3> kMultiStdDevs = (RobotBase.isReal()) ?
        VecBuilder.fill(0.01, 0.01, 5.0) : VecBuilder.fill(0.01, 0.01, 5.0);

    public static final double kAmbiguityThreshold = (RobotBase.isReal()) ? 0.2 : 1.0;
}