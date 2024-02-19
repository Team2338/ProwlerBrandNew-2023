package team.gif.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.Constants;
import team.gif.robot.Robot;
import team.gif.robot.RobotMap;

public class Shooter extends SubsystemBase {
	private static Shooter instance = null;
	
	public static Shooter getInstance() {
		if (instance == null) {
			instance = new Shooter();
		}
		return instance;
	}
	
//	private static final CANSparkMax flywheelMotor = new CANSparkMax(RobotMap.FLYWHEEL, CANSparkMaxLowLevel.MotorType.kBrushless);
//	private static final CANPIDController flywheelPIDController = flywheelMotor.getPIDController();
//	private static final CANEncoder flywheelEncoder = flywheelMotor.getEncoder();

	private static final TalonSRX flywheelMotor = new TalonSRX(RobotMap.FLYWHEEL);

	
	private Shooter() {
		super();
		flywheelMotor.configFactoryDefault();
		flywheelMotor.setNeutralMode(NeutralMode.Coast);
    }
	
	public void setSpeed(double percent) {
		flywheelMotor.set(TalonSRXControlMode.PercentOutput, percent);
	}
	
}
