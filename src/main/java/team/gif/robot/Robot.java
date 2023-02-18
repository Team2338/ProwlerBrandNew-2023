// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import team.gif.lib.autoMode;
import team.gif.robot.commands.arm.ArmPIDControl;
import team.gif.lib.logging.EventFileLogger;
import team.gif.lib.logging.TelemetryFileLogger;
import team.gif.robot.commands.drivetrain.DriveArcade;
import team.gif.robot.commands.drivetrain.DriveSwerve;
import team.gif.robot.commands.elevator.ElevatorPIDControl;
import team.gif.robot.commands.arm.ArmManualControl;
import team.gif.robot.commands.elevator.ElevatorManualControl;
import team.gif.robot.subsystems.Arm;
import team.gif.robot.subsystems.Collector;
import team.gif.robot.subsystems.CollectorPneumatics;
import team.gif.robot.subsystems.Drivetrain;
import team.gif.robot.subsystems.Elevator;
import team.gif.robot.subsystems.SwerveDrivetrain;
import team.gif.robot.subsystems.TelescopingArm;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;
    private static TelemetryFileLogger telemetryLogger;
    public static EventFileLogger eventLogger;
    public static Drivetrain drivetrain;
    public static DriveArcade arcadeDrive;
    public static SwerveDrivetrain swervetrain = null;
    public static DriveSwerve driveSwerve;
    public static Arm arm;
    public static Elevator elevator;
    public static Collector collector;
    public static CollectorPneumatics collectorPneumatics;
    public static TelescopingArm telescopingArm;
    public static OI oi;
    public static UiSmartDashboard uiSmartDashboard;

    public static UI ui;

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        eventLogger = new EventFileLogger();
        eventLogger.init();

        telemetryLogger = new TelemetryFileLogger();

        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        m_robotContainer = new RobotContainer();
        arm = new Arm();
        elevator = new Elevator();
        collector = new Collector();
        collectorPneumatics = new CollectorPneumatics();
        telescopingArm = new TelescopingArm();
        ui = new UI();
        uiSmartDashboard = new UiSmartDashboard();

        if (isSwervePBot || isCompBot) {
            swervetrain = new SwerveDrivetrain();
            driveSwerve = new DriveSwerve();
            swervetrain.setDefaultCommand(driveSwerve);
            swervetrain.resetHeading();
        } else {
            drivetrain = new Drivetrain(false, false);
            arcadeDrive = new DriveArcade();
            drivetrain.setDefaultCommand(arcadeDrive);
        }
//        arm.setDefaultCommand(new ArmManualControl());
        arm.setTargetPosition(arm.getPosition());
        arm.setDefaultCommand(new ArmPIDControl());

        elevator.setElevatorTargetPos(elevator.getPosition());
        elevator.setDefaultCommand(new ElevatorPIDControl());
//        elevator.setDefaultCommand(new ElevatorManualControl());

        // settings default wheels to WheelsIn;
        collectorPneumatics.pneumaticsIn();

        oi = new OI();
        telescopingArm.setDefaultCommand(new MotorRun());

        if (isSwervePBot || isCompBot) {
            ShuffleboardTab swerveTab = Shuffleboard.getTab("Swerve");
            swerveTab.addDouble("robot x", swervetrain.getRobotPose()::getX);
            swerveTab.addDouble("robot y", swervetrain.getRobotPose()::getY);
            swerveTab.addDouble("robot rot", swervetrain.getRobotPose().getRotation()::getDegrees);
        }

        SmartDashboard.putNumber("Collector Speed",.70);

        telemetryLogger.init();
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();

        uiSmartDashboard.updateUI();

    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
    @Override
    public void autonomousInit() {
        if (isSwervePBot) {
            m_autonomousCommand = m_robotContainer.getAutonomousCommand(autoMode.SWERVE_POC);
        } else {
            m_autonomousCommand = m_robotContainer.getAutonomousCommand(null);
        }
//        m_autonomousCommand = m_robotContainer.getAutonomousCommand(chosenAuto);


        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        double timeLeft = DriverStation.getMatchTime();
        oi.setRumble((timeLeft <= 40.0 && timeLeft >= 36.0) ||
                (timeLeft <= 5.0 && timeLeft >= 3.0));

        telemetryLogger.run();
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {}

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {}

    private void addMetricsToLogger() {
        telemetryLogger.addMetric("TimeStamp", Timer::getFPGATimestamp);

        telemetryLogger.addMetric("Driver_Left_Y", () -> -Robot.oi.driver.getLeftY());
        telemetryLogger.addMetric("Driver_Right_X", () -> Robot.oi.driver.getRightX());
    }

    //TODO: Change and check before each usage
    public static boolean isCompBot = false;
    public static boolean isSwervePBot = false;
    public static boolean isTankPBot = true;
}
