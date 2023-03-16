package team.gif.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.Robot;
import team.gif.robot.RobotMap;

public class Indexer extends SubsystemBase {
	private static Indexer instance = null;
	
	public static Indexer getInstance() {
		if (instance == null) {
			instance = new Indexer();
		}
		return instance;
	}
	
	
	// Victors
	private static final VictorSPX stageTwoMotorVictor   = new VictorSPX(RobotMap.STAGE_TWO);
	private static final VictorSPX stageThreeMotorVictor = new VictorSPX(RobotMap.STAGE_THREE);
	private static final VictorSPX stageFourMotorVictor  = new VictorSPX(RobotMap.STAGE_FOUR);
	private static final VictorSPX stageFiveMotorVictor  = new VictorSPX(RobotMap.STAGE_FIVE);
	
	private static final DigitalInput knopfSensor = new DigitalInput(RobotMap.SENSOR_KNOPF);
	
	private static final DigitalInput stageOneSensor   = new DigitalInput(RobotMap.SENSOR_ONE);
	private static final DigitalInput stageTwoSensor   = new DigitalInput(RobotMap.SENSOR_TWO);
	private static final DigitalInput stageThreeSensor = new DigitalInput(RobotMap.SENSOR_THREE);
	private static final DigitalInput stageFourSensor  = new DigitalInput(RobotMap.SENSOR_FOUR);
	private static final DigitalInput stageFiveSensor  = new DigitalInput(RobotMap.SENSOR_FIVE);
	
	
	private Indexer() {
		super();

		stageTwoMotorVictor.setNeutralMode(  NeutralMode.Brake);
		stageThreeMotorVictor.setNeutralMode(NeutralMode.Brake);
		stageFourMotorVictor.setNeutralMode( NeutralMode.Brake);
		stageFiveMotorVictor.setNeutralMode( NeutralMode.Brake);
		
		stageFourMotorVictor.setInverted(Robot.isCompBot); // C:false P:true
		
	}
	
	public boolean getKnopf() {
		return knopfSensor.get();
	}
	
	public boolean[] getState() {
		boolean[] sensorStates = {false, stageOneSensor.get(), stageTwoSensor.get(), stageThreeSensor.get(), stageFourSensor.get(), stageFiveSensor.get()};
		return sensorStates;
	}
	
	public void setSpeedTwo(double speed) {
		stageTwoMotorVictor.set(ControlMode.PercentOutput, speed);
	}
	public void setSpeedThree(double speed) {
		stageThreeMotorVictor.set(ControlMode.PercentOutput, speed);
	}
	public void setSpeedFour(double speed) {
		stageFourMotorVictor.set(ControlMode.PercentOutput, speed);
	}
	public void setSpeedFive(double speed) {
		stageFiveMotorVictor.set(ControlMode.PercentOutput, speed);
	}
}
