package frc.robot.util;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class MathUtil {
  public static double wrapToCircle(double angle, double fullCircle) {
    angle %= fullCircle;
    return angle < 0 ? fullCircle + angle : angle;
  }

  public static boolean isWithinTolerance(double value, double target, double tolerance) {
    return Math.abs(value - target) < tolerance;
  }

  public static double clampAngle(double angle) {
    angle = MathUtil.wrapToCircle(angle, 2 * Math.PI);
    if (angle > Math.PI) {
      angle -= 2 * Math.PI;
    }
    return angle;
  }

  public static double fitDeadband(double val, double deadband) {
    if (Math.abs(val) >= deadband) {
      if (val > 0) {
        if (val >= 1) {
          return 1;
        } else {
          return (val - deadband) / (1 - deadband);
        }
      } else if (val < 0) {
        if (val <= -1) {
          return -1;
        } else {
          return (val + deadband) / (1 - deadband);
        }
      }
    }
    return 0;
  }

  public static double flipAngleOverYAxis(double angle) {
    double radians = Math.toRadians(angle);
    double sin = Math.sin(radians);
    double cos = Math.cos(radians);
    double mirroredCos = -cos;
    double mirroredRadians = Math.atan2(sin, mirroredCos);
    double mirroredAngle = Math.toDegrees(mirroredRadians);
    return mirroredAngle;
  }

  public static double powPreserveSign(double a, double b) {
    final double sign = Math.signum(a);
    return sign * Math.pow(Math.abs(a), b);
  }

  public static double[] circleVectorFromSquare(double x, double y, double vectorScaleFactor) {
    double angle = Math.atan2(y, x);
    double maxDist = (0.375 + ((Math.asin(Math.sin(2 * (angle + 0.785398163397) + 1.57079632679)))
        / (Math.sin(2 * (angle + 0.785398163397) + 1.57079632679) * 1.57079632679)));
    double scaleRatio = vectorScaleFactor * Math.sqrt(x * x + y * y) / maxDist;
    return new double[] { x * scaleRatio, y * scaleRatio };
  }

  public static ChassisSpeeds limitXAndYAcceleration(ChassisSpeeds targetChassisSpeeds,
      ChassisSpeeds currentChassisSpeeds, double maxAccelerationX, double maxAccelerationY, double loopTime) {
    double targetAccelerationX = (targetChassisSpeeds.vxMetersPerSecond - currentChassisSpeeds.vxMetersPerSecond)
        / loopTime;
    double targetAccelerationY = (targetChassisSpeeds.vyMetersPerSecond - currentChassisSpeeds.vyMetersPerSecond)
        / loopTime;

    if (Math.abs(targetAccelerationX) > maxAccelerationX) {
      targetAccelerationX = maxAccelerationX * Math.signum(targetAccelerationX);
    }

    if (Math.abs(targetAccelerationY) > maxAccelerationY) {
      targetAccelerationY = maxAccelerationY * Math.signum(targetAccelerationY);
    }

    return new ChassisSpeeds(currentChassisSpeeds.vxMetersPerSecond + targetAccelerationX * loopTime,
        currentChassisSpeeds.vyMetersPerSecond + targetAccelerationY * loopTime,
        targetChassisSpeeds.omegaRadiansPerSecond);

  }
}
