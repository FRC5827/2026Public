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
    private LoggedDashboardChooser<String> locationChooser1;
    private LoggedDashboardChooser<String> locationChooser2;
    private LoggedDashboardChooser<String> locationChooser3;
    private LoggedDashboardChooser<Integer> delayChooser;
    private LoggedDashboardChooser<String> sysIdChooser;
    private SwitchableChooser autoChooser1;
    private SwitchableChooser autoChooser2;
    private SwitchableChooser autoChooser3;
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
            locationChooser1 =
                    new LoggedDashboardChooser<>(
                            "Starting Location", new SendableChooser<String>());

            autoChooser1 = new SwitchableChooser("Auto Chooser 1");
            autoChooser2 = new SwitchableChooser("Auto Chooser 2");
            autoChooser3 = new SwitchableChooser("Auto Chooser 3");
            locationChooser1.addDefaultOption("Left Trench", "Left Trench");
            locationChooser1.addOption("Left Bump", "Left Bump");
            locationChooser1.addOption("Right Bump", "Right Bump");
            locationChooser1.addOption("Right Trench", "Right Trench");
            locationChooser2 =
                    new LoggedDashboardChooser<>(
                            "Starting Location 2", new SendableChooser<String>());
            locationChooser2.addDefaultOption("No 2nd auto!", "No 2nd auto!");
            locationChooser2.addOption("Left Bump", "Left Bump");
            locationChooser2.addOption("Left Trench", "Left Trench");
            locationChooser2.addOption("Right Bump", "Right Bump");
            locationChooser2.addOption("Right Trench", "Right Trench");
            locationChooser3 =
                    new LoggedDashboardChooser<>(
                            "Starting Location 3", new SendableChooser<String>());
            locationChooser3.addDefaultOption("No 3rd auto!", "No 3rd auto!");
            locationChooser3.addOption("Left Bump", "Left Bump");
            locationChooser3.addOption("Left Trench", "Left Trench");
            locationChooser3.addOption("Right Bump", "Right Bump");
            locationChooser3.addOption("Right Trench", "Right Trench");
            // Initialize location maps for all location choosers
            this.autoNames.put("Left Bump", new ArrayList<>());
            this.autoNames.put("Left Trench", new ArrayList<>());
            this.autoNames.put("Right Bump", new ArrayList<>());
            this.autoNames.put("Right Trench", new ArrayList<>());

            List<String> autoNames = AutoBuilder.getAllAutoNames();
            for (String autoName : autoNames) {
                if (autoName.startsWith("Left Bump")) {
                    this.autoNames.get("Left Bump").add(autoName);
                } else if (autoName.startsWith("Left Trench")) {
                    this.autoNames.get("Left Trench").add(autoName);
                } else if (autoName.startsWith("Right Bump")) {
                    this.autoNames.get("Right Bump").add(autoName);
                } else if (autoName.startsWith("Right Trench")) {
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
            locationChooser1
                    .getSendableChooser()
                    .onChange(
                            (s) -> {
                                // s is the location string from the callback
                                String locStr = s != null ? s : "Left Trench";
                                List<String> options = AutoChooser.this.autoNames.get(locStr);
                                if (options != null && !options.isEmpty()) {
                                    autoChooser1.setOptions(options.toArray(new String[0]));
                                    Elastic.sendNotification(
                                            new Elastic.Notification(
                                                    Elastic.NotificationLevel.INFO,
                                                    "Location Changed",
                                                    "Your selected location has changed."));
                                }
                            });
            locationChooser2
                    .getSendableChooser()
                    .onChange(
                            (s) -> {
                                // s is the location string from the callback
                                String locStr = s != null ? s : "Left Trench";
                                List<String> options = AutoChooser.this.autoNames.get(locStr);
                                if ("No 2nd auto!".equals(s)) {
                                    autoChooser2.setOptions(new String[] {"No 2nd auto!"});
                                    Elastic.sendNotification(
                                            new Elastic.Notification(
                                                    Elastic.NotificationLevel.INFO,
                                                    "No 2nd Auto Selected",
                                                    "You have selected to run only one auto routine. If you want to run two auto routines, make sure you select a starting location for the second auto."));
                                } else if (options != null && !options.isEmpty()) {
                                    autoChooser2.setOptions(options.toArray(new String[0]));
                                    Elastic.sendNotification(
                                            new Elastic.Notification(
                                                    Elastic.NotificationLevel.INFO,
                                                    "Location Changed",
                                                    "Your selected location has changed."));
                                }
                            });
            locationChooser3
                    .getSendableChooser()
                    .onChange(
                            (s) -> {
                                // s is the location string from the callback
                                String locStr = s != null ? s : "Left Trench";
                                List<String> options = AutoChooser.this.autoNames.get(locStr);
                                if ("No 3rd auto!".equals(s)) {
                                    autoChooser3.setOptions(new String[] {"No 3rd auto!"});
                                    Elastic.sendNotification(
                                            new Elastic.Notification(
                                                    Elastic.NotificationLevel.INFO,
                                                    "No 3rd Auto Selected",
                                                    "You have selected to run only one or two auto routines. If you want to run three auto routines, make sure you select a starting location for the third auto."));
                                } else if (options != null && !options.isEmpty()) {
                                    autoChooser3.setOptions(options.toArray(new String[0]));
                                    Elastic.sendNotification(
                                            new Elastic.Notification(
                                                    Elastic.NotificationLevel.INFO,
                                                    "Location Changed",
                                                    "Your selected location has changed."));
                                }
                            });

            // Set initial options
            String initialLoc = locationChooser1.get();
            if (initialLoc == null) {
                initialLoc = "Left Trench";
            }
            List<String> initialOptions = AutoChooser.this.autoNames.get(initialLoc);
            if (initialOptions != null && !initialOptions.isEmpty()) {
                autoChooser1.setOptions(initialOptions.toArray(new String[0]));
            }
            // Initialize second auto chooser based on its location selection
            String initialLoc2 = locationChooser2.get();
            if (initialLoc2 == null) {
                initialLoc2 = "Left Trench";
            }
            List<String> initialOptions2 = AutoChooser.this.autoNames.get(initialLoc2);
            if ("No 2nd auto!".equals(initialLoc2)) {
                autoChooser2.setOptions(new String[] {"No 2nd auto!"});
            } else if (initialOptions2 != null && !initialOptions2.isEmpty()) {
                autoChooser2.setOptions(initialOptions2.toArray(new String[0]));
            }
            // Initialize third auto chooser based on its location selection
            String initialLoc3 = locationChooser3.get();
            if (initialLoc3 == null) {
                initialLoc3 = "Left Trench";
            }
            List<String> initialOptions3 = AutoChooser.this.autoNames.get(initialLoc3);
            if ("No 3rd auto!".equals(initialLoc3)) {
                autoChooser3.setOptions(new String[] {"No 3rd auto!"});
            } else if (initialOptions3 != null && !initialOptions3.isEmpty()) {
                autoChooser3.setOptions(initialOptions3.toArray(new String[0]));
            }
            Elastic.sendNotification(
                    new Notification()
                            .withTitle(
                                    "IMPORTANT: Change each individual location for each auto, even if it's correct!")
                            .withDescription(
                                    "Unexpected behavior may result otherwise. Select different location, and then change it back, even if same location on EACH auto!")
                            .withLevel(NotificationLevel.WARNING)
                            .withNoAutoDismiss());
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
            // Log robot state
            Logger.recordOutput("AutoChooser/CharacterizationMode", Constants.characterizationMode);

            // Build the current auto name safely using strings only
            int delay = delayChooser.get() != null ? delayChooser.get() : 0;
            Logger.recordOutput("AutoChooser/Delay", delay);
            if (Constants.characterizationMode) {
                String selected = sysIdChooser.get();
                autoName = delay + "+" + (selected != null ? selected : "None");

            } else {
                String selectedAuto1 = autoChooser1.get();
                String selectedAuto2 = autoChooser2.get();
                String selectedAuto3 = autoChooser3.get();
                if (selectedAuto1 == null || selectedAuto1.equals("No auto!")) {
                    autoName = "No auto!";
                } else if (selectedAuto2 == null || selectedAuto2.equals("No 2nd auto!")) {
                    autoName = delay + "+" + selectedAuto1 + "+" + "No 2nd auto!";
                } else if (selectedAuto3 == null || selectedAuto3.equals("No 3rd auto!")) {
                    autoName =
                            delay
                                    + "+"
                                    + selectedAuto1
                                    + "+"
                                    + selectedAuto2
                                    + "+"
                                    + "No 3rd auto!";
                } else {
                    autoName =
                            delay + "+" + selectedAuto1 + "+" + selectedAuto2 + "+" + selectedAuto3;
                }
            }

            // If the auto name has changed since the last loop, update the auto routine and send a
            // notification
            boolean autoNameChanged = !previousAutoName.equals(autoName);
            Logger.recordOutput("AutoChooser/PreviousAutoName", previousAutoName);
            Logger.recordOutput("AutoChooser/NewAutoName", autoName);
            Logger.recordOutput("AutoChooser/AutoNameChanged", autoNameChanged);
            if (autoNameChanged) {
                if (Constants.characterizationMode) {
                    String selectedSysId = sysIdChooser.get();
                    Logger.recordOutput(
                            "AutoChooser/SysIdSelected",
                            selectedSysId != null ? selectedSysId : "null");
                    Logger.recordOutput(
                            "AutoChooser/SysIdRoutineKeys",
                            sysIdRoutines.keySet().toArray(new String[0]));
                    if (selectedSysId == null || "None".equals(selectedSysId)) {
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
                    String loc1 = locationChooser1 != null ? locationChooser1.get() : "null";
                    String loc2 = locationChooser2 != null ? locationChooser2.get() : "null";
                    String loc3 = locationChooser3 != null ? locationChooser3.get() : "null";
                    Logger.recordOutput(
                            "AutoChooser/LocationChooser1", loc1 != null ? loc1 : "null");
                    Logger.recordOutput(
                            "AutoChooser/LocationChooser2", loc2 != null ? loc2 : "null");
                    Logger.recordOutput(
                            "AutoChooser/LocationChooser3", loc3 != null ? loc3 : "null");
                    // Log SwitchableChooser 1 internals
                    if (autoChooser1 != null) {
                        String raw1 = autoChooser1.getSelectedRaw();
                        Logger.recordOutput(
                                "AutoChooser/AutoChooser1/SelectedRaw",
                                raw1 != null ? raw1 : "null");
                        Logger.recordOutput("AutoChooser/AutoChooser1/Active", autoChooser1.get());
                        Logger.recordOutput(
                                "AutoChooser/AutoChooser1/Options", autoChooser1.getOptions());
                    }
                    // Log SwitchableChooser 2 internals
                    if (autoChooser2 != null) {
                        String raw2 = autoChooser2.getSelectedRaw();
                        Logger.recordOutput(
                                "AutoChooser/AutoChooser2/SelectedRaw",
                                raw2 != null ? raw2 : "null");
                        Logger.recordOutput("AutoChooser/AutoChooser2/Active", autoChooser2.get());
                        Logger.recordOutput(
                                "AutoChooser/AutoChooser2/Options", autoChooser2.getOptions());
                    }
                    // Log SwitchableChooser 3 internals
                    if (autoChooser3 != null) {
                        String raw3 = autoChooser3.getSelectedRaw();
                        Logger.recordOutput(
                                "AutoChooser/AutoChooser3/SelectedRaw",
                                raw3 != null ? raw3 : "null");
                        Logger.recordOutput("AutoChooser/AutoChooser3/Active", autoChooser3.get());
                        Logger.recordOutput(
                                "AutoChooser/AutoChooser3/Options", autoChooser3.getOptions());
                    }
                    if (autoName.contains("No auto!")) {
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
                    } else if (autoName.contains("No 2nd auto!")) {
                        Elastic.sendNotification(
                                new Notification()
                                        .withTitle("Only one auto selected")
                                        .withDescription(
                                                "Only one auto selected. If you want to run two autos, make sure you set a second auto")
                                        .withLevel(NotificationLevel.INFO));
                        String selectedAuto1 = autoChooser1.get();
                        auto =
                                Commands.waitSeconds(delay)
                                        .andThen(new PathPlannerAuto(selectedAuto1));

                    } else if (autoName.contains("No 3rd auto!")) {
                        Elastic.sendNotification(
                                new Notification()
                                        .withTitle("Multiple autos selected")
                                        .withDescription(
                                                "Multiple autos selected. Make sure that's your intent! If not, change the location for the second and third auto choosers to something, then change back to \"No 2nd auto!\" or \"No 3rd auto!\".")
                                        .withLevel(NotificationLevel.WARNING)
                                        .withDisplaySeconds(15));
                        Elastic.sendNotification(
                                new Notification()
                                        .withTitle("Only two autos selected")
                                        .withDescription(
                                                "Only two autos selected. If you want to run three autos, make sure you set a third auto")
                                        .withLevel(NotificationLevel.INFO)
                                        .withDisplaySeconds(10));

                        String selectedAuto1 = autoChooser1.get();
                        String selectedAuto2 = autoChooser2.get();
                        auto =
                                Commands.waitSeconds(delay)
                                        .andThen(new PathPlannerAuto(selectedAuto1))
                                        .andThen(new PathPlannerAuto(selectedAuto2));
                    } else {
                        String selectedAuto1 = autoChooser1.get();
                        String selectedAuto2 = autoChooser2.get();
                        String selectedAuto3 = autoChooser3.get();
                        autoName =
                                delay
                                        + "+"
                                        + selectedAuto1
                                        + "+"
                                        + selectedAuto2
                                        + "+"
                                        + selectedAuto3;
                        Elastic.sendNotification(
                                new Notification()
                                        .withTitle("Multiple autos selected")
                                        .withDescription(
                                                "Multiple autos selected. Make sure that's your intent! If not, change the location for the second and third auto choosers to something, then change back to \"No 2nd auto!\" or \"No 3rd auto!\".")
                                        .withLevel(NotificationLevel.WARNING)
                                        .withDisplaySeconds(15));
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
                            if (selectedAuto2 == null || selectedAuto2.equals("No 2nd auto!")) {
                                auto =
                                        Commands.waitSeconds(delay)
                                                .andThen(new PathPlannerAuto(selectedAuto1));
                            } else if (selectedAuto3 == null
                                    || selectedAuto3.equals("No 3rd auto!")) {
                                auto =
                                        Commands.waitSeconds(delay)
                                                .andThen(new PathPlannerAuto(selectedAuto1))
                                                .andThen(new PathPlannerAuto(selectedAuto2));
                            } else {
                                auto =
                                        Commands.waitSeconds(delay)
                                                .andThen(new PathPlannerAuto(selectedAuto1))
                                                .andThen(new PathPlannerAuto(selectedAuto2))
                                                .andThen(new PathPlannerAuto(selectedAuto3));
                            }
                            ;
                        } catch (Exception e) {
                            // This should never happen, but if it does, we want to catch the
                            // exception and send a notification instead of crashing the robot code,
                            // because that would be bad. If this happens, we'll just set the auto
                            // routine to "do nothing" to be safe.
                            Elastic.sendNotification(
                                    new Notification()
                                            .withTitle("Error loading auto")
                                            .withDescription(
                                                    "Error loading auto: " + e.getMessage())
                                            .withLevel(NotificationLevel.ERROR)
                                            .withNoAutoDismiss());
                            e.printStackTrace();
                            auto = Commands.none();
                        }
                        auto = auto.withName(autoName);
                    }
                }
            }
            Logger.recordOutput(
                    "PerformanceMonitor/AutoChooser",
                    (Timer.getFPGATimestamp() - startTime) * 1000);
        }
    }
}
