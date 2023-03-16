package team.gif.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Globals;
import team.gif.robot.subsystems.Indexer;
import team.gif.robot.subsystems.Intake;
import team.gif.robot.subsystems.Shooter;

public class RapidFire extends CommandBase {
	
	public RapidFire() {
		addRequirements(Shooter.getInstance());
	}
	
	@Override
	public void initialize() {
//		Globals.indexerEnabled = false;
	}
	
	@Override
	public void execute() {
		Shooter.getInstance().setVoltage(10);
		Indexer.getInstance().setSpeedFive(0.5);
		Indexer.getInstance().setSpeedFour(0.4);
		Indexer.getInstance().setSpeedThree(0.3);
		Indexer.getInstance().setSpeedTwo(0.3);
		Intake.getInstance().setSpeed(0.3);
	}
	
	@Override
	public boolean isFinished() {
		return false;
	}
	
	@Override
	public void end(boolean interrupted) {
		Shooter.getInstance().setVoltage(0);
		Indexer.getInstance().setSpeedFive(0);
		Indexer.getInstance().setSpeedFour(0);
		Indexer.getInstance().setSpeedThree(0);
		Indexer.getInstance().setSpeedTwo(0);
		Intake.getInstance().setSpeed(0);
		
		Globals.indexerEnabled = true;
	}
}
