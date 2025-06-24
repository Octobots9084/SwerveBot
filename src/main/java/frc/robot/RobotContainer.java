package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.SwerveIO;
import frc.robot.Subsystems.Swerve.SwerveIOSystem;
import org.ironmaple.simulation.SimulatedArena;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;

public class RobotContainer {
  private final SendableChooser<Command> autoChooser;
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


    //AUTOS//////////////
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // NamedCommands.registerCommand("test", swerve.autoBalanceCommand());

    //new EventTrigger("shoot note").and(new Trigger(exampleSubsystem::someCondition)).onTrue(Commands.print("shoot note");

    //new PointTowardsZoneTrigger("Speaker").whileTrue(Commands.print("aiming at speaker"));

    // autoCommand = new PathPlannerAuto("Example Auto");
  // // PathPlannerAuto can also be created with a custom command
  // // autoCommand = new PathPlannerAuto(new CustomAutoCommand());

  // // Bind to different auto triggers
  // autoCommand.isRunning().onTrue(Commands.print("Example Auto started"));
  // autoCommand.timeElapsed(5).onTrue(Commands.print("5 seconds passed"));
  // autoCommand.timeRange(6, 8).whileTrue(Commands.print("between 6 and 8 seconds"));
  // autoCommand.event("Example Event Marker").onTrue(Commands.print("passed example event marker"));
  // autoCommand.pointTowardsZone("Speaker").onTrue(Commands.print("aiming at speaker"));
  // autoCommand.activePath("Example Path").onTrue(Commands.print("started following Example Path"));
  // autoCommand.nearFieldPosition(new Translation2d(2, 2), 0.5).whileTrue(Commands.print("within 0.5m of (2, 2)"));
  // autoCommand.inFieldArea(new Translation2d(2, 2), new Translation2d(4, 4)).whileTrue(Commands.print("in area of (2, 2) - (4, 4)"));
















    // configureButtonBindings();
  }


  //AUTOS/////////


  public Command getAutonomousCommand() {
    try{
        // Load the path you want to follow using its name in the GUI
        PathPlannerPath path = PathPlannerPath.fromPathFile("Example Path");

        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.followPath(path);
    } catch (Exception e) {
        DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
        return Commands.none();
    }
  }


  // public Command followPathCommand(String pathName) {
  //   try{
  //       PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

  //       return new FollowPathCommand(
  //               path,
  //               this::getPose, // Robot pose supplier
  //               this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
  //               this::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds, AND feedforwards
  //               new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
  //                       new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
  //                       new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
  //               ),
  //               Constants.robotConfig, // The robot configuration
  //               () -> {
  //                 // Boolean supplier that controls when the path will be mirrored for the red alliance
  //                 // This will flip the path being followed to the red side of the field.
  //                 // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

  //                 var alliance = DriverStation.getAlliance();
  //                 if (alliance.isPresent()) {
  //                   return alliance.get() == DriverStation.Alliance.Red;
  //                 }
  //                 return false;
  //               },
  //               this // Reference to this subsystem to set requirements
  //       );
  //   } catch (Exception e) {
  //       DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
  //       return Commands.none();
  //   }
  // }
}
