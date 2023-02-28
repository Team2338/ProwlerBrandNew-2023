package team.gif.lib;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

/**
 * Singleton class for creating a trajectory for a swerve bot
 * @author Rohan Cherukuri
 * @since 2/14/22
 */
public class RobotTrajectory {
    public RobotTrajectory() {}

    private static RobotTrajectory instance = null;

    public static RobotTrajectory getInstance() {
        if(instance == null) {
            instance = new RobotTrajectory();
        }
        return instance;
    }

//    public TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
//        Constants.AutoConstants.MAX_SPEED_METERS_PER_SECOND,
//        Constants.AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED
//    )
//        .setKinematics(Constants.Drivetrain.DRIVE_KINEMATICS);
//
}