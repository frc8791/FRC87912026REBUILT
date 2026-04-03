package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.SwerveSubsystem;

public final class Autos {

  public static Command exampleAuto(SwerveSubsystem drivebase) {
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile("StraightTest");
      return AutoBuilder.followPath(path);
    } catch (Exception e) {
      DriverStation.reportError("Failed to load auto path: " + e.getMessage(), e.getStackTrace());
      return Commands.none();
    }
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}