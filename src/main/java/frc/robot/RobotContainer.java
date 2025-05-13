package frc.robot;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.SwerveIO;
import frc.robot.Subsystems.Swerve.SwerveIOSystem;
import org.ironmaple.simulation.SimulatedArena;

public class RobotContainer {

  private Swerve swerve;

  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        Swerve.setInstance(new SwerveIOSystem());
        swerve = Swerve.getInstance();

        break;

      case SIM:
        Swerve.setInstance(new SwerveIOSystem());
        swerve = Swerve.getInstance();

        SimulatedArena.getInstance().resetFieldForAuto();
        break;

      case REPLAY:
        Swerve.setInstance(new SwerveIO() {});
        swerve = Swerve.getInstance();
        break;
      default:
        break;
    }

    TeleopDrive closedFieldRel =
        new TeleopDrive(
            () ->
                MathUtil.applyDeadband(
                    -ButtonConfig.driverLeft.getRawAxis(1), OperatorConstants.LEFT_Y_DEADBAND),
            () ->
                MathUtil.applyDeadband(
                    -ButtonConfig.driverLeft.getRawAxis(0), OperatorConstants.LEFT_X_DEADBAND),
            () ->
                MathUtil.applyDeadband(
                    -ButtonConfig.driverRight.getRawAxis(0), OperatorConstants.RIGHT_X_DEADBAND));
    swerve.setDefaultCommand(closedFieldRel);

    ButtonConfig buttons = new ButtonConfig();
    buttons.initTeleop();
  }
}
