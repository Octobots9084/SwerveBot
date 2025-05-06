package frc.robot.Subsystems.Swerve;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class SwerveIOInputsAutoLogged extends SwerveIO.SwerveIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("Pose", pose);
    table.put("Speeds", speeds);
    table.put("TargetSpeeds", targetSpeeds);
    table.put("SwerveModuleStates", swerveModuleStates);
    table.put("SwerveModuleDesiredStates", swerveModuleDesiredStates);
  }

  @Override
  public void fromLog(LogTable table) {
    pose = table.get("Pose", pose);
    speeds = table.get("Speeds", speeds);
    targetSpeeds = table.get("TargetSpeeds", targetSpeeds);
    swerveModuleStates = table.get("SwerveModuleStates", swerveModuleStates);
    swerveModuleDesiredStates = table.get("SwerveModuleDesiredStates", swerveModuleDesiredStates);
  }

  public SwerveIOInputsAutoLogged clone() {
    SwerveIOInputsAutoLogged copy = new SwerveIOInputsAutoLogged();
    copy.pose = this.pose;
    copy.speeds = this.speeds;
    copy.targetSpeeds = this.targetSpeeds;
    copy.swerveModuleStates = this.swerveModuleStates.clone();
    copy.swerveModuleDesiredStates = this.swerveModuleDesiredStates.clone();
    return copy;
  }
}
