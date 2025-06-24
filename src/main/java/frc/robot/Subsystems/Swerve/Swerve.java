package frc.robot.Subsystems.Swerve;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

public class Swerve extends SubsystemBase {
  SwerveIO io;
  private final SwerveIOInputsAutoLogged inputs = new SwerveIOInputsAutoLogged();
  private static Swerve INSTANCE = null;

  public static enum DriveState {
    None,
    Manual
  };

  private DriveState driveState = DriveState.None;

  public boolean isAlignedToSource;
  public boolean isAlignedToCoralRight;
  public boolean isAlignedToCoralLeft;
  public boolean isAlignedToProcessor;
  public boolean isAlignedCenterReef;

  public static Swerve getInstance() {
    if (INSTANCE == null) {
      throw new IllegalStateException("Swerve instance not set");
    }
    return INSTANCE;
  }

  public static Swerve setInstance(SwerveIO io) {
    INSTANCE = new Swerve(io);
    return INSTANCE;
  }

  public Optional<Pose2d> getSimPose() {
    return io.getSimPose();
  }

  public Swerve(SwerveIO io) {
    this.io = io;

    RobotConfig config;
    try{
      config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
            config,
            () -> {
              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this
    );
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  public SwerveIO getIo() {
    return io;
  }

  public void zeroGyro() {
    this.io.zeroGyro();
  }

  @Override
  public void periodic() {
    this.io.updateInputs(inputs);
    Logger.processInputs("Swerve Drive", inputs);
  }

  public double getGyro() {
    return this.io.getGyro();
  }

  public Pose2d getPose() {
    return this.io.getPose();
  }

  public void resetPose(Pose2d pose) {
    this.io.resetPose(pose);
  }

  public void setDriveState(DriveState state) {
    driveState = state;
    Logger.recordOutput("DriveState", state);
  }

  public DriveState getDriveState() {
    return driveState;
  }

  public ChassisSpeeds getSpeeds() {
    return this.io.getSpeeds();
  }

  public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
    this.io.driveFieldRelative(fieldRelativeSpeeds);
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    SmartDashboard.putString("Chassis speeds", robotRelativeSpeeds.toString());
    this.io.driveRobotRelative(robotRelativeSpeeds);
  }

  public Command driveCommand(
      DoubleSupplier translationX,
      DoubleSupplier translationY,
      DoubleSupplier headingX,
      DoubleSupplier headingY) {
    return run(
        () -> {
          double xInput = Math.pow(translationX.getAsDouble(), 3); // Smooth control out
          double yInput = Math.pow(translationY.getAsDouble(), 3); // Smooth control out
          // Make the robot move
          driveFieldRelative(
              this.io.getTargetSpeeds(
                  xInput, yInput, headingX.getAsDouble(), headingY.getAsDouble()));
        });
  }

  public Command driveCommand(
      DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX) {
    return run(
        () -> {
          // Make the robot move
          this.io.driveRobotRelative(
              new Translation2d(
                  translationX.getAsDouble() * this.io.getMaximumChassisVelocity(),
                  translationY.getAsDouble() * this.io.getMaximumChassisVelocity()),
              angularRotationX.getAsDouble() * this.io.getMaximumChassisAngularVelocity(),
              true,
              false);
        });
  }

  public void addVisionReading(
      Pose2d robotPose, double timestamp, Matrix<N3, N1> visionMeasurementStdDevs) {
    this.io.addVisionReading(robotPose, timestamp, visionMeasurementStdDevs);
  }
}
