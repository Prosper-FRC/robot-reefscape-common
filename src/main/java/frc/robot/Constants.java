package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {

    // AdvantageKit modes
    public static enum Mode {
        REAL,
        SIM,
        REPLAY
    }

    public static final Mode kCurrentMode = 
    // Mode.REPLAY;
    RobotBase.isReal() ? Mode.REAL : Mode.SIM;
    // Set Tuning to true during development, false during competition
    public static final boolean kTuningMode = true;

    // ROBOT SEPCIFIC
    public static final String kCanbusName = "drivebase";

    public static final AprilTagFieldLayout kFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    public static final double kFieldLengthMeters = kFieldLayout.getFieldLength();
    public static final double kFieldWidthMeters = kFieldLayout.getFieldWidth();

    public static final int kAprilTagCount = 22;

    public static final double kLoopPeriod = 0.02;
}
