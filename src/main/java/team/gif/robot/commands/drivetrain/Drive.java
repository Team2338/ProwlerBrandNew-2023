
package team.gif.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Robot;
import team.gif.robot.subsystems.Drivetrain;

/**
 * An example command that uses an example subsystem.
 */
public class Drive extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    //private final Drivetrain m_subsystem;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public Drive() {
        //m_subsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(Robot.drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        /**
         *                Arcade Drive
         */
        // Y-axis on X-box controller is negative when pushed away
        double currSpeed = -Robot.oi.driver.getLeftY();
        double currRotation = Robot.oi.driver.getRightX();
        Robot.drivetrain.driveArcade(currSpeed, currRotation);

        /**
         *                Tank Drive
         */
        // Y-axis on X-box controller is negative when pushed away

//        double currLeft = -Robot.oi.driver.getLeftY();
//        double currRight = Robot.oi.driver.getRightY();
//        Robot.drivetrain.setSpeed(currLeft, currRight);

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.drivetrain.driveArcade(0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
