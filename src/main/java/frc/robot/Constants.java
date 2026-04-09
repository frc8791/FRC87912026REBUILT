package frc.robot;

import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.util.Units;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double DEADBAND = 0.05;
  }

  public static final double maxSpeed = Units.feetToMeters(7.0);

  public static final RobotConfig robotConfig;

  static {
    try {
      robotConfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      throw new RuntimeException("Failed to load PathPlanner RobotConfig from GUI settings", e);
    }
  }
}