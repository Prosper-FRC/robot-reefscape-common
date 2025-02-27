package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class FieldConstants {
    public static final Pose2d kReefCenter = new Pose2d(4.5, 4, Rotation2d.fromDegrees(0.0));

    public static final double kXNetLineMeters = 7.15;

    public static final Pose2d AL = new Pose2d(3.18, 4.03, Rotation2d.k180deg);
    public static final Pose2d AR = new Pose2d(3.18, 3.64, Rotation2d.k180deg);

    public static final Pose2d BL = new Pose2d(3.78, 5.13, Rotation2d.fromDegrees(120.0));
    public static final Pose2d BR = new Pose2d(3.52, 4.97, Rotation2d.fromDegrees(120.0));

    public static final Pose2d CL = new Pose2d(5.08, 5.20, Rotation2d.fromDegrees(60.0));
    public static final Pose2d CR = new Pose2d(4.77, 5.38, Rotation2d.fromDegrees(60.0));

    public static final Pose2d DL = new Pose2d(5.80, 4.01, Rotation2d.kZero);
    public static final Pose2d DM = new Pose2d(5.80, 4.09, Rotation2d.kZero);
    public static final Pose2d DR = new Pose2d(5.80, 4.47, Rotation2d.kZero);

    public static final Pose2d EL = new Pose2d(4.82, 2.71, Rotation2d.fromDegrees(-60.0));
    public static final Pose2d EM = new Pose2d(5.13, 2.90, Rotation2d.fromDegrees(-60.0));
    public static final Pose2d ER = new Pose2d(5.10, 2.89, Rotation2d.fromDegrees(-60.0));

    public static final Pose2d FL = new Pose2d(3.54, 3.05, Rotation2d.fromDegrees(-120.0));
    public static final Pose2d FR = new Pose2d(3.81, 2.92, Rotation2d.fromDegrees(-120.0));

    public static final Pose2d IL = new Pose2d(1.6, 7.42, Rotation2d.fromRadians(2.20));
    public static final Pose2d IR = new Pose2d(1.57, 0.66, Rotation2d.fromRadians(-2.20));
}
