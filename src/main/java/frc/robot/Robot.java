// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
    private Command autonomousCommand;
    private RobotContainer robotContainer;

    // Track running commands via scheduler callbacks
    private final Set<Command> activeCommands = new LinkedHashSet<>();
    // Track commands executed this cycle (populated by onCommandExecute, cleared each periodic)
    private final List<String> executedThisCycle = new ArrayList<>();

    public Robot() {
        // Record metadata
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        Logger.recordMetadata(
                "GitDirty",
                switch (BuildConstants.DIRTY) {
                    case 0 -> "All changes committed";
                    case 1 -> "Uncommitted changes";
                    default -> "Unknown";
                });

        // Set up data receivers & replay source
        switch (Constants.currentMode) {
            case REAL:
                // Running on a real robot, log to a USB stick ("/U/logs")
                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());
                break;

            case SIM:
                // Running a physics simulator, log to NT
                Logger.addDataReceiver(new NT4Publisher());
                break;

            case REPLAY:
                // Replaying a log, set up replay source
                setUseTiming(false); // Run as fast as possible
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(
                        new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
                Logger.addDataReceiver(new NT4Publisher());
                break;
        }

        // Start AdvantageKit logger
        Logger.start();

        // Register CommandScheduler callbacks for logging only in sim/replay to avoid impacting
        // real robot performance
        if (Constants.currentMode == Constants.simMode) {
            CommandScheduler.getInstance()
                    .onCommandInitialize(
                            cmd -> {
                                activeCommands.add(cmd);
                                Logger.recordOutput(
                                        "CommandScheduler/Events/Initialized", cmd.getName());
                            });
            CommandScheduler.getInstance()
                    .onCommandExecute(
                            cmd -> {
                                executedThisCycle.add(
                                        cmd.getName()
                                                + " [requires: "
                                                + cmd.getRequirements().stream()
                                                        .map(s -> s.getName())
                                                        .reduce((a, b) -> a + ", " + b)
                                                        .orElse("")
                                                + "] (class: "
                                                + cmd.getClass().getSimpleName()
                                                + ")");
                            });
            CommandScheduler.getInstance()
                    .onCommandFinish(
                            cmd -> {
                                activeCommands.remove(cmd);
                                Logger.recordOutput(
                                        "CommandScheduler/Events/Finished", cmd.getName());
                            });
            CommandScheduler.getInstance()
                    .onCommandInterrupt(
                            cmd -> {
                                activeCommands.remove(cmd);
                                Logger.recordOutput(
                                        "CommandScheduler/Events/Interrupted", cmd.getName());
                            });
        }

        FollowPathCommand.warmupCommand().schedule();
        // For Elastic layout
        WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our autonomous chooser on the dashboard.
        robotContainer = new RobotContainer();
    }

    /** This function is called periodically during all modes. */
    @Override
    public void robotPeriodic() {
        // Optionally switch the thread to high priority to improve loop
        // timing (see the template project documentation for details)
        Threads.setCurrentThreadPriority(true, 99);

        long loopStart = RobotController.getFPGATime();

        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled commands, running already-scheduled commands, removing
        // finished or interrupted commands, and running subsystem periodic() methods.
        // This must be called from the robot's periodic block in order for anything in
        // the Command-based framework to work.
        CommandScheduler.getInstance().run();

        // Log the entire CommandScheduler state if sim or replay (to avoid impacting real robot
        // performance)
        if (Constants.currentMode == Constants.simMode) {
            logCommandScheduler();
        }

        // Check trench alignment and trigger controller rumble if aligned
        robotContainer.checkTrenchAlignment();

        // Log total loop time
        Logger.recordOutput(
                "PerformanceMonitor/LoopCycleMs",
                (RobotController.getFPGATime() - loopStart) / 1000.0);

        // Return to non-RT thread priority (do not modify the first argument)
        Threads.setCurrentThreadPriority(false, 10);
    }

    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(autonomousCommand);
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {}

    /** This function is called once when test mode is enabled. */
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

    @Override
    public void disabledInit() {
        robotContainer.disable();
    }

    @Override
    public void disabledExit() {
        robotContainer.enable();
    }

    /** Logs the full state of the CommandScheduler to AdvantageKit. */
    private void logCommandScheduler() {
        // Log currently active commands (tracked via callbacks)
        String[] names = activeCommands.stream().map(Command::getName).toArray(String[]::new);
        String[] details =
                activeCommands.stream()
                        .map(
                                cmd ->
                                        cmd.getName()
                                                + " [requires: "
                                                + cmd.getRequirements().stream()
                                                        .map(s -> s.getName())
                                                        .reduce((a, b) -> a + ", " + b)
                                                        .orElse("")
                                                + "] (class: "
                                                + cmd.getClass().getSimpleName()
                                                + ")")
                        .toArray(String[]::new);

        Logger.recordOutput("CommandScheduler/RunningCommands", names);
        Logger.recordOutput("CommandScheduler/RunningCommandDetails", details);
        Logger.recordOutput("CommandScheduler/RunningCommandCount", names.length);

        // Log every command that executed this cycle (from onCommandExecute callback)
        Logger.recordOutput(
                "CommandScheduler/ExecutedThisCycle", executedThisCycle.toArray(new String[0]));
        Logger.recordOutput("CommandScheduler/ExecutedThisCycleCount", executedThisCycle.size());
        executedThisCycle.clear();
    }
}
