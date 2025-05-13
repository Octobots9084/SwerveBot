package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Subsystems.Swerve.Swerve;

public class ButtonConfig {
  static CommandJoystick driverLeft = ControlMap.DRIVER_LEFT;
  static CommandJoystick driverRight = ControlMap.DRIVER_RIGHT;
  static CommandJoystick driverButtons = ControlMap.DRIVER_BUTTONS;
  static CommandJoystick coDriverLeft = ControlMap.CO_DRIVER_LEFT;
  static CommandJoystick coDriverRight = ControlMap.CO_DRIVER_RIGHT;
  static CommandJoystick coDriverButtons = ControlMap.CO_DRIVER_BUTTONS;

  public void initTeleop() {
    driverButtons
        .button(6)
        .onTrue(
            new InstantCommand(
                () -> {
                  Swerve.getInstance().zeroGyro();
                }));
  }
}
