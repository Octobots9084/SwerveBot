package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.util.MathUtil;
import java.util.function.DoubleSupplier;

public class TeleopDrive extends Command {
  private final DoubleSupplier vX;
  private final DoubleSupplier vY;
  private final DoubleSupplier omega;
  private static Swerve swerveInstance = Swerve.getInstance();

  public TeleopDrive(DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier omega) {
    this.vX = vX;
    this.vY = vY;
    this.omega = omega;
    this.addRequirements(Swerve.getInstance());
  }

  @Override
  public void initialize() {
    SmartDashboard.putString("Command", "command");
  }

  @Override
  public void execute() {
    switch (swerveInstance.getDriveState()) {
      case Manual:
        SmartDashboard.putString("the code works!!", "nope sorry");
        double[] speeds = MathUtil.circleVectorFromSquare(vX.getAsDouble(), vY.getAsDouble(), swerveInstance.getIo().getMaxSpeed()); 
        Swerve.getInstance()
            .driveFieldRelative(
                new ChassisSpeeds(
                    speeds[0],
                    speeds[1],
                    omega.getAsDouble() * swerveInstance.getIo().getMaxTurnSpeed()));
        break;
      case AlignReef:
        SmartDashboard.putString("AlignReef", "AllignReef");
      case AlignProcessor:
        break;
      case AlignSource:
        break;
      default:
        SmartDashboard.putString("the code works!!", "nope sorry");
        Swerve.getInstance()
            .driveFieldRelative(
                new ChassisSpeeds(
                    vX.getAsDouble() * swerveInstance.getIo().getMaxSpeed(),
                    vY.getAsDouble() * swerveInstance.getIo().getMaxSpeed(),
                    omega.getAsDouble() * swerveInstance.getIo().getMaxTurnSpeed()));
        break;
    }
  }
}
