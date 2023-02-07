package team.gif.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.Constants;
import team.gif.robot.RobotMap;

public class Arm extends SubsystemBase {
    public static WPI_TalonSRX armMotor = new WPI_TalonSRX(RobotMap.ARM_MOTOR);
    private static WPI_TalonSRX armEncoderTalon = new WPI_TalonSRX(RobotMap.ARM_ENCODER);
    private static MotorController armControl;

    private static final int MAX_SUPPLY_CURRENT_AMPS = 20;
    private static final int MAX_STATOR_CURRENT_AMPS = 90;

    public Arm() {
        //motor controller groups
//        armControl = new MotorControllerGroup(armMotor);

        currentLimitingEnable(true); //limits

        //armMotor settings
        armMotor.setNeutralMode(NeutralMode.Brake); //setting to brake mode
        armMotor.configFactoryDefault();
        armMotor.setInverted(true); //maybe we might change to true (IDK)
        armMotor.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 20);
        armMotor.enableCurrentLimit(false); //limiter
        armMotor.setSensorPhase(true);

        //armEncoderTalon settings
        armEncoderTalon.setNeutralMode(NeutralMode.Brake); //setting to brake mode
        armEncoderTalon.configFactoryDefault();
        armEncoderTalon.setInverted(true); //maybe we might change to true (IDK)
        armEncoderTalon.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 20);
        armEncoderTalon.enableCurrentLimit(false); //limiter
        armEncoderTalon.setSensorPhase(true);

        armMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

        // PID
        armMotor.config_kF(0, Constants.Arm.FF); //feed forward
        armMotor.config_kP(0, Constants.Arm.P); //proportional
        armMotor.config_kI(0,Constants.Arm.I);
        armMotor.config_IntegralZone(0,200);
    }

    // This method will set ticks that the arm have to move.
    public void setAngle(double ticks) {
        while (getTicks() < ticks) {
//            armEncoderTalon.set(ticks);
        }
    }

    // getting the ticks from the encoders.
    public double getTicks() {
        return armMotor.getSelectedSensorPosition(); // 4096
//        return armMotor.getSupplyCurrent();
    }

    public double getTicksRel(){
        return armEncoderTalon.getSelectedSensorPosition();
    }

    // getting the ticks from the encoders.
    public void move(double percent) {
        if( (percent > 0 && getTicks() < Constants.Arm.TICKS_ABS_MAX) ||
            (percent < 0 && getTicks() > Constants.Arm.TICKS_ABS_MIN)
        ) {
            armMotor.set(percent);
        }
        else
            armMotor.set(0);

    }

    public void PIDMove(double position) {
        armMotor.set(ControlMode.Position, position);
    }

    public double PIDError(){
        return armMotor.getClosedLoopError();
    }

    // getting the ticks from the encoders.
    public double getTicks2() {
        return armEncoderTalon.getSelectedSensorPosition();
    }

    // limits
    public void currentLimitingEnable(boolean enableLimit) {
        armMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(enableLimit, MAX_SUPPLY_CURRENT_AMPS,MAX_STATOR_CURRENT_AMPS, 0));
    }


}