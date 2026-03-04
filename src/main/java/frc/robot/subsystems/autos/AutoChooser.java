package frc.robot.subsystems.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.Constants;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.NotificationLevel;
import frc.robot.util.SwitchableChooser;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class AutoChooser extends SubsystemBase {
    private LoggedDashboardChooser<String> locationChooser;
    private LoggedDashboardChooser<Integer> delayChooser;
    private LoggedDashboardChooser<String> sysIdChooser;
    private SwitchableChooser compAutoChooser;
    private Drive drive;
    private Command auto;
    private Map<String, List<String>> autoNames;
    private Map<String, Command> sysIdRoutines = new HashMap<>();
    private String autoName;

    public AutoChooser(Drive drive) {
        this.drive = drive;
        this.auto = Commands.none();
        this.autoNames = new HashMap<>();
        this.autoName = "No auto!";
        buildAutoChooser();
    }

    private void buildAutoChooser() {
        // Set up auto routines
        delayChooser = new LoggedDashboardChooser<>("Auto Delay", new SendableChooser<>());
        delayChooser.addDefaultOption("No delay", 0);
        for (int i = 1; i <= 15; i++) {
            delayChooser.addOption(i + " seconds", i);
        }
        if (Constants.characterizationMode) {
            sysIdChooser = new LoggedDashboardChooser<>("SysID Routines", new SendableChooser<>());
            sysIdChooser.addDefaultOption("None", "None");
            sysIdRoutines.put("None", Commands.none());
            // Set up SysId routines
            sysIdChooser.addOption(
                    "Drive Wheel Radius Characterization", "Drive Wheel Radius Characterization");
            sysIdRoutines.put(
                    "Drive Wheel Radius Characterization",
                    DriveCommands.wheelRadiusCharacterization(drive));

            sysIdChooser.addOption(
                    "Drive Simple FF Characterization", "Drive Simple FF Characterization");
            sysIdRoutines.put(
                    "Drive Simple FF Characterization",
                    DriveCommands.feedforwardCharacterization(drive));

            sysIdChooser.addOption(
                    "Drive SysId (Quasistatic Forward)", "Drive SysId (Quasistatic Forward)");
            sysIdRoutines.put(
                    "Drive SysId (Quasistatic Forward)",
                    drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));

            sysIdChooser.addOption(
                    "Drive SysId (Quasistatic Reverse)", "Drive SysId (Quasistatic Reverse)");
            sysIdRoutines.put(
                    "Drive SysId (Quasistatic Reverse)",
                    drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

            sysIdChooser.addOption(
                    "Drive SysId (Dynamic Forward)", "Drive SysId (Dynamic Forward)");
            sysIdRoutines.put(
                    "Drive SysId (Dynamic Forward)",
                    drive.sysIdDynamic(SysIdRoutine.Direction.kForward));

            sysIdChooser.addOption(
                    "Drive SysId (Dynamic Reverse)", "Drive SysId (Dynamic Reverse)");
            sysIdRoutines.put(
                    "Drive SysId (Dynamic Reverse)",
                    drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
            Elastic.sendNotification(
                    new Notification()
                            .withTitle("Characterization Mode")
                            .withDescription(
                                    "Ask code team if you're confused. Match starting? Set to 'No auto' & complain later, as comp auto WON'T BE RUN. Know what you're doing? Disregard.")
                            .withLevel(NotificationLevel.WARNING)
                            .withNoAutoDismiss());
        } else {
            locationChooser =
                    new LoggedDashboardChooser<>(
                            "Starting Location", new SendableChooser<String>());

            compAutoChooser = new SwitchableChooser("Comp Auto Chooser");

            locationChooser.addDefaultOption("Left Bump", "Left Bump");
            locationChooser.addOption("Left Trench", "Left Trench");
            locationChooser.addOption("Right Bump", "Right Bump");
            locationChooser.addOption("Right Trench", "Right Trench");

            // Initialize location maps
            this.autoNames.put("Left Bump", new ArrayList<>());
            this.autoNames.put("Left Trench", new ArrayList<>());
            this.autoNames.put("Right Bump", new ArrayList<>());
            this.autoNames.put("Right Trench", new ArrayList<>());

            List<String> autoNames = AutoBuilder.getAllAutoNames();
            for (String autoName : autoNames) {
                if (autoName.contains("Left Bump")) {
                    this.autoNames.get("Left Bump").add(autoName);
                } else if (autoName.contains("Left Trench")) {
                    this.autoNames.get("Left Trench").add(autoName);
                } else if (autoName.contains("Right Bump")) {
                    this.autoNames.get("Right Bump").add(autoName);
                } else if (autoName.contains("Right Trench")) {
                    this.autoNames.get("Right Trench").add(autoName);
                }
            }

            // Add "No auto!" to all lists
            for (List<String> list : this.autoNames.values()) {
                list.add("No auto!");
            }
            // Ensures that when the location is changed, the auto options are updated accordingly.
            // The callback function takes in the new location string as a parameter, retrieves the
            // corresponding auto routine options from the autoNames map, and updates the
            // compAutoChooser with those options. It also sends a notification to Elastic to inform
            // the user that the location has been changed and they should select a new auto routine
            // based on the new location.
            locationChooser
                    .getSendableChooser()
                    .onChange(
                            (s) -> {
                                // s is the location string from the callback
                                String locStr = s != null ? s : "Left Bump";
                                List<String> options = AutoChooser.this.autoNames.get(locStr);
                                if (options != null && !options.isEmpty()) {
                                    compAutoChooser.setOptions(options.toArray(new String[0]));
                                    Elastic.sendNotification(
                                            new Elastic.Notification(
                                                    Elastic.NotificationLevel.INFO,
                                                    "Location Changed",
                                                    "Your selected location has changed."));
                                }
                            });

            // Set initial options
            String initialLoc = locationChooser.get();
            if (initialLoc == null) {
                initialLoc = "Left Bump";
            }
            List<String> initialOptions = AutoChooser.this.autoNames.get(initialLoc);
            if (initialOptions != null && !initialOptions.isEmpty()) {
                compAutoChooser.setOptions(initialOptions.toArray(new String[0]));
            }
            Elastic.sendNotification(
                    new Notification()
                            .withTitle("Auto Chooser Initialized in Competition Mode")
                            .withDescription(
                                    "The auto chooser has been initialized with the available competition autos. Please select a starting location and auto routine.")
                            .withLevel(NotificationLevel.INFO)
                            .withDisplaySeconds(30));
        }
    }

    public Command get() {
        return auto;
    }

    @Override
    public void periodic() {
        double startTime = Timer.getFPGATimestamp();
        // Only update the auto routine when disabled for two reasons:
        // 1. It prevents accidentally changing the auto routine while robot is enabled
        // 2. It prevents loop overruns while the robot is enabled
        if (RobotState.isDisabled()) {
            String previousAutoName = autoName;

            // Build the current auto name safely using strings only
            int delay = delayChooser.get() != null ? delayChooser.get() : 0;
            if (Constants.characterizationMode) {
                String selected = sysIdChooser.get();
                autoName = delay + "+" + (selected != null ? selected : "None");
            } else {
                String selected = compAutoChooser.get();
                if (selected == null || selected.equals("No auto!")) {
                    autoName = "No auto!";
                } else {
                    autoName = delay + "+" + selected;
                }
            }
            // If the auto name has changed since the last loop, update the auto routine and send a
            // notification
            if (!previousAutoName.equals(autoName)) {

                if (Constants.characterizationMode) {
                    String selectedSysId = sysIdChooser.get();
                    if (selectedSysId == null || selectedSysId.equals("None")) {
                        Elastic.sendNotification(
                                new Notification()
                                        .withTitle("No SysId routine selected")
                                        .withDescription(
                                                "No SysId routine will be run. If that is not your intent, make sure you set a SysId routine")
                                        .withLevel(NotificationLevel.WARNING)
                                        .withNoAutoDismiss());
                        auto = Commands.none();
                    } else {
                        // Similar to the competition auto routines, the SysId routine command
                        // consists of two components:
                        // 1. A command that waits for the specified delay time before starting the
                        // SysId routine (Why will you want to delay a SysId routine? I don't know,
                        // but we should support it just in case. Maybe you want to run a SysId
                        // routine in the middle of a match or something, who am I to judge?) Delay
                        // is supported for comp autos anyway, so might as well support it for SysId
                        // routines too for consistency)
                        // 2. The actual SysId routine command, which is retrieved from the
                        // sysIdRoutines map using the selected SysId routine string as the key.
                        Command routine = sysIdRoutines.get(selectedSysId);
                        auto =
                                Commands.waitSeconds(delay)
                                        .andThen(routine != null ? routine : Commands.none());
                        Elastic.sendNotification(
                                new Notification()
                                        .withTitle("SysId routine changed")
                                        .withDescription("SysId routine changed to " + autoName)
                                        .withLevel(NotificationLevel.INFO));
                    }
                } else {
                    if (autoName.equals("No auto!")) {
                        // Because they are no more leave points or stupid Auto RPs, it doesn't make
                        // sense to create a
                        // default auto that just drives forward and stops after a few seconds, so
                        // we'll just do nothing for "No auto!" In the event leave points are added
                        // back in the future, check the 2025 codebase for an example of how to
                        // create a default auto routine that drives forward and stops after a few
                        // seconds.
                        Elastic.sendNotification(
                                new Notification()
                                        .withTitle("No auto selected")
                                        .withDescription(
                                                "No auto will be run. If that's is not your intent, make sure you set an auto")
                                        .withLevel(NotificationLevel.WARNING)
                                        .withNoAutoDismiss());
                        auto = Commands.none();
                    } else {
                        String selectedPath = compAutoChooser.get();
                        Elastic.sendNotification(
                                new Notification()
                                        .withTitle("Auto changed")
                                        .withDescription("Auto changed to " + autoName)
                                        .withLevel(NotificationLevel.INFO));
                        try {
                            // The auto command fundamentally consists of two components:
                            // 1. A command that waits for the specified delay time before starting
                            // the auto routine
                            // 2. The actual auto routine command, which is a PathPlannerAuto object
                            // constructed with the selected auto, based on the string, so make sure
                            // they're the same
                            auto =
                                    Commands.waitSeconds(delay)
                                            .andThen(new PathPlannerAuto(selectedPath));
                            Elastic.sendNotification(
                                    new Notification()
                                            .withTitle("Auto loaded")
                                            .withDescription(
                                                    "Auto '"
                                                            + selectedPath
                                                            + "' loaded successfully.")
                                            .withLevel(NotificationLevel.INFO));
                        } catch (Exception e) {
                            // This should never happen, but if it does, we want to catch the
                            // exception and send a notification instead of crashing the robot code,
                            // because that would be bad. If this happens, we'll just set the auto
                            // routine to "do nothing" to be safe.
                            Elastic.sendNotification(
                                    new Notification()
                                            .withTitle("Error loading auto")
                                            .withDescription(
                                                    "Error loading auto '"
                                                            + selectedPath
                                                            + "': "
                                                            + e.getMessage())
                                            .withLevel(NotificationLevel.ERROR)
                                            .withNoAutoDismiss());
                            e.printStackTrace();
                            auto = Commands.none();
                        }
                    }
                }
            }
        }
        Logger.recordOutput(
                "PerformanceMonitor/AutoChooser", (Timer.getFPGATimestamp() - startTime) * 1000);
    }
}
