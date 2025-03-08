package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class FieldConstants {
    public static final Pose2d kReefCenter = new Pose2d(4.48249, Constants.kFieldWidthMeters / 2.0, Rotation2d.fromDegrees(0.0));

    public static final double kXNetLineMeters = 7.15;

    public static final Pose2d AL = new Pose2d(3.22, 4.39, Rotation2d.k180deg);
    public static final Pose2d AR = new Pose2d(3.22, 4.08, Rotation2d.k180deg);

    public static final Pose2d BL = new Pose2d(4.193, 5.318, Rotation2d.fromDegrees(120.0));
    public static final Pose2d BR = new Pose2d(3.927, 5.165, Rotation2d.fromDegrees(120.0));

    public static final Pose2d CL = new Pose2d(5.433, 4.944, Rotation2d.fromDegrees(60.0));
    public static final Pose2d CR = new Pose2d(5.174, 5.105, Rotation2d.fromDegrees(60.0));

    public static final Pose2d DL = new Pose2d(5.76, 3.94, Rotation2d.kZero);
    public static final Pose2d DM = new Pose2d(0.0, 4.09, Rotation2d.kZero);
    public static final Pose2d DR = new Pose2d(5.76, 3.62, Rotation2d.kZero);

    public static final Pose2d FL = new Pose2d(3.53, 3.10, Rotation2d.fromDegrees(-120.0));
    public static final Pose2d FR = new Pose2d(3.81, 2.94, Rotation2d.fromDegrees(-120.0));

    public static final Pose2d EL = new Pose2d(4.80, 2.73, Rotation2d.fromDegrees(-60.0));
    public static final Pose2d EM = new Pose2d(5.13, 2.90, Rotation2d.fromDegrees(-60.0));
    public static final Pose2d ER = new Pose2d(5.05, 2.88, Rotation2d.fromDegrees(-60.0));

    public static final Pose2d redIL = new Pose2d(1.632, 7.42, Rotation2d.fromRadians(2.20).plus(Rotation2d.k180deg).plus(Rotation2d.k180deg));
    public static final Pose2d redIR = new Pose2d(1.31, 0.95, Rotation2d.fromRadians(-2.20).plus(Rotation2d.k180deg).plus(Rotation2d.k180deg));

    public static final Pose2d blueIL = new Pose2d(1.1, 7.11, Rotation2d.fromRadians(2.20).plus(Rotation2d.k180deg).plus(Rotation2d.k180deg));
    public static final Pose2d blueIR = new Pose2d(1.33, 0.78, Rotation2d.fromRadians(-2.20).plus(Rotation2d.k180deg).plus(Rotation2d.k180deg));
}
