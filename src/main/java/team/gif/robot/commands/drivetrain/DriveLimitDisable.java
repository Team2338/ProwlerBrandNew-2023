
package team.gif.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Robot;
import team.gif.robot.subsystems.Drivetrain;

/**
 * An example command that uses an example subsystem.
 */
public class DriveLimitDisable extends CommandBase {

    public DriveLimitDisable() {
        //m_subsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        //addRequirements(Drivetrain.getInstance());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        Robot.drivetrain.currentLimitingEnable(false);
        System.out.println("drive limit override enabled");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.drivetrain.currentLimitingEnable(true);
        System.out.println("drive limit override disabled sponsored by geico");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
