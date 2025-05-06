package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static class OperatorConstants {
    public static final double LEFT_X_DEADBAND = 0.07;
    public static final double LEFT_Y_DEADBAND = 0.07;
    public static final double RIGHT_X_DEADBAND = 0.07;
    public static final double TURN_CONSTANT = 6;
    public static final int DRIVER_LEFT = 0;
    public static final int DRIVER_RIGHT = 1;
    public static final int DRIVER_BUTTONS = 2;
    public static final int CO_DRIVER_LEFT = 3;
    public static final int CO_DRIVER_RIGHT = 4;
    public static final int CO_DRIVER_BUTTONS = 5;
  }

  public static enum Mode {
    REAL,
    SIM,
    REPLAY
  }
}
