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
    private Runnable listener;
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
        locationChooser =
                new LoggedDashboardChooser<>("Starting Location", new SendableChooser<String>());
        sysIdChooser = new LoggedDashboardChooser<>("SysID Routines", new SendableChooser<>());
        compAutoChooser = new SwitchableChooser("Comp Auto Chooser");
        sysIdChooser.addDefaultOption("None", "None");
        sysIdRoutines.put("None", Commands.none());
        delayChooser.addDefaultOption("No delay", 0);
        for (int i = 1; i <= 15; i++) {
            delayChooser.addOption(i + " seconds", i);
        }
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

        sysIdChooser.addOption("Drive SysId (Dynamic Forward)", "Drive SysId (Dynamic Forward)");
        sysIdRoutines.put(
                "Drive SysId (Dynamic Forward)",
                drive.sysIdDynamic(SysIdRoutine.Direction.kForward));

        sysIdChooser.addOption("Drive SysId (Dynamic Reverse)", "Drive SysId (Dynamic Reverse)");
        sysIdRoutines.put(
                "Drive SysId (Dynamic Reverse)",
                drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        locationChooser
                .getSendableChooser()
                .onChange(
                        (s) -> {
                            // s is the location string from the callback
                            String locStr = s != null ? s : "Left Bump";
                            List<String> options = AutoChooser.this.autoNames.get(locStr);
                            options.add("No auto!"); // Ensure "No auto!" is always an option
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
    }

    public Command get() {
        return auto;
    }

    public void onChange(Runnable listener) {
        this.listener = listener;
    }

    @Override
    public void periodic() {
        double startTime = Timer.getFPGATimestamp();
        if (RobotState.isDisabled()) {
            String previousAutoName = autoName;

            // Build the current auto name safely using strings only
            int delay = delayChooser.get() != null ? delayChooser.get() : 0;
            if (!Constants.characterizationMode) {
                String selected = compAutoChooser.get();
                if (selected == null || selected.equals("No auto!")) {
                    autoName = "No auto!";
                } else {
                    autoName = delay + "+" + selected;
                }
            } else {
                String selected = sysIdChooser.get();
                autoName = delay + "+" + (selected != null ? selected : "None");
            }

            if (!previousAutoName.equals(autoName)) {

                if (!Constants.characterizationMode) {
                    if (autoName.equals("No auto!")) {
                        // Because they are no more leave points, it doesn't make sense to create a
                        // default auto that just drives forward and stops after a few seconds, so
                        // we'll just do nothing for "No auto!"
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
                } else {
                    String selectedSysId = sysIdChooser.get();
                    if (selectedSysId.equals("None") || selectedSysId == null) {
                        Elastic.sendNotification(
                                new Notification()
                                        .withTitle("No SysId routine selected")
                                        .withDescription(
                                                "No SysId routine will be run. If that is not your intent, make sure you set a SysId routine")
                                        .withLevel(NotificationLevel.WARNING)
                                        .withNoAutoDismiss());
                        auto = Commands.none();
                    } else {
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
                }

                if (listener != null) {
                    listener.run();
                }
            }
        }
        Logger.recordOutput(
                "PeriodicTime/AutoChooser", (Timer.getFPGATimestamp() - startTime) * 1000);
    }
}
