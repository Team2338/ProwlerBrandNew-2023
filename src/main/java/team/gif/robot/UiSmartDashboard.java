package team.gif.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team.gif.lib.autoMode;
import team.gif.lib.delay;
import team.gif.robot.commands.drivetrain.ResetHeading;

public class UiSmartDashboard {

    public SendableChooser<autoMode> autoModeChooser = new SendableChooser<>();
    public SendableChooser<delay> delayChooser = new SendableChooser<>();

    public UiSmartDashboard() {
        ShuffleboardTab tab = Shuffleboard.getTab("SmartDashboard"); // Gets a reference to the shuffleboard tab
        tab.add("BotHead", (x) -> {
                x.setSmartDashboardType("Gyro");
                x.addDoubleProperty("Value", () -> Robot.pigeon.getCompassHeading(), null);
            })
            .withPosition(5, 0);

        // Auto selections
        autoModeChooser.addOption("Swerve POC", autoMode.SWERVE_POC);
        autoModeChooser.addOption("Engage", autoMode.ENGAGE);
        autoModeChooser.addOption("Place, Collect", autoMode.PLACE_COLLECT);
        autoModeChooser.setDefaultOption("Place, Engage", autoMode.PLACE_ENGAGE);
        autoModeChooser.addOption("Place, Mobility", autoMode.PLACE_MOBILITY);
        autoModeChooser.addOption("Place, Mobility, Engage", autoMode.PLACE_MOBILITY_ENGAGE);
        autoModeChooser.addOption("Place, Collect, Place", autoMode.PLACE_COLLECT_PLACE);

        tab.add("Auto Select", autoModeChooser)
            .withWidget(BuiltInWidgets.kComboBoxChooser)
            .withPosition(7, 1)
            .withSize(2, 1);

        // Auto delay
        delayChooser.setDefaultOption("0", delay.DELAY_0);
        delayChooser.addOption("1", delay.DELAY_1);
        delayChooser.addOption("2", delay.DELAY_2);
        delayChooser.addOption("3", delay.DELAY_3);
        delayChooser.addOption("4", delay.DELAY_4);
        delayChooser.addOption("5", delay.DELAY_5);
        delayChooser.addOption("6", delay.DELAY_6);
        delayChooser.addOption("7", delay.DELAY_7);
        delayChooser.addOption("8", delay.DELAY_8);
        delayChooser.addOption("9", delay.DELAY_9);
        delayChooser.addOption("10", delay.DELAY_10);
        delayChooser.addOption("11", delay.DELAY_11);
        delayChooser.addOption("12", delay.DELAY_12);
        delayChooser.addOption("13", delay.DELAY_13);
        delayChooser.addOption("14", delay.DELAY_14);
        delayChooser.addOption("15", delay.DELAY_15);

        SmartDashboard.putData("Reset", new ResetHeading());

        tab.add("Delay", delayChooser)
            .withPosition(7, 0)
            .withSize(1, 1);
    }

    public void updateUI() {
        // Timers
        SmartDashboard.putString("Time", String.format("%.4f", Timer.getFPGATimestamp()));
        SmartDashboard.putString("Timer",String.format("%.2f",Timer.getMatchTime()));
    }
}
