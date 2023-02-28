package team.gif.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Robot;

public class LimelightLEDControl extends CommandBase {

    public LimelightLEDControl() {
    }

    @Override
    public void initialize() {
        Robot.limelight.setLEDMode(3);
    }

    @Override
    public void execute() {

    }


    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Robot.limelight.setLEDMode(1);
    }
}
