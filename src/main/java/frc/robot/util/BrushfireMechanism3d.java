package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class BrushfireMechanism3d {
  private static double EXTENSION_0_HEIGHT = 0.48;
  private static double EXTENSION_1_MAX = 0.568;
  private static double EXTENSION_2_MAX = 0.603;

  public static double MIN_EXTENSION = 0.529;
  public static double MAX_EXTENSION = MIN_EXTENSION + EXTENSION_1_MAX + EXTENSION_2_MAX;

  /**
   * Calculates the component poses for rendering the state of Brushfire's arm in AdvantageScope.
   * This poses from this method should be logged periodically.
   *
   * @param bottomPivotAngle The angle of the bottom pivot where 0° is horizontal
   * @param extensionMeters The distance between the bottom pivot point and the top pivot point
   * @param topPivotAngle The angle of the top pivot where 0° is straight
   * @return The 3D component poses
   */
  public static Pose3d[] getPoses(
      Rotation2d bottomPivotAngle, double extensionMeters, Rotation2d topPivotAngle) {
    extensionMeters = MathUtil.clamp(extensionMeters, MIN_EXTENSION, MAX_EXTENSION);
    double extensionFraction =
        MathUtil.inverseInterpolate(MIN_EXTENSION, MAX_EXTENSION, extensionMeters);

    Pose3d extension0Pose =
        new Pose3d(
            0.0, 0.0, EXTENSION_0_HEIGHT, new Rotation3d(0.0, -bottomPivotAngle.getRadians(), 0.0));
    Pose3d extension1Pose =
        extension0Pose.transformBy(
            new Transform3d(EXTENSION_1_MAX * extensionFraction, 0.0, 0.0, new Rotation3d()));
    Pose3d extension2Pose =
        extension1Pose.transformBy(
            new Transform3d(EXTENSION_2_MAX * extensionFraction, 0.0, 0.0, new Rotation3d()));
    Pose3d rollersPose =
        extension0Pose.transformBy(
            new Transform3d(
                extensionMeters, 0.0, 0.0, new Rotation3d(0.0, -topPivotAngle.getRadians(), 0.0)));

    return new Pose3d[] {extension0Pose, extension1Pose, extension2Pose, rollersPose};
  }

  private BrushfireMechanism3d() {}
}
