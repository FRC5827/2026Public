package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import java.util.Optional;

public class GameTimeUtil {
    private static final double BUFFER_TIME = 3.0;

    /*
     * Determines whether the hub is active based on the current match time and game data.
     * @param time The current match time in seconds.
     * @return true if the hub is active, false otherwise.
     */
    public static boolean isHubActive(double time) {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        // If we have no alliance, we cannot be enabled, therefore no hub.
        if (alliance.isEmpty()) {
            return false;
        }
        // Hub is always enabled in autonomous.
        if (DriverStation.isAutonomousEnabled()) {
            return true;
        }
        // At this point, if we're not teleop enabled, keep shooting
        if (!DriverStation.isTeleopEnabled()) {
            return true;
        }

        // We're teleop enabled, compute.
        String gameData = DriverStation.getGameSpecificMessage();
        // If we have no game data, we cannot compute, assume hub is active, as its likely early in
        // teleop.
        if (gameData.isEmpty()) {
            return true;
        }
        boolean redInactiveFirst = false;
        switch (gameData.charAt(0)) {
            case 'R' -> redInactiveFirst = true;
            case 'B' -> redInactiveFirst = false;
            default -> {
                // If we have invalid game data, assume hub is active.
                return true;
            }
        }

        // Shift was is active for blue if red won auto, or red if blue won auto.
        boolean shift1Active =
                switch (alliance.get()) {
                    case Red -> !redInactiveFirst;
                    case Blue -> redInactiveFirst;
                };

        // 3 second buffer time where both teams can shoot.

        if (time > 130) {
            // Transition shift, hub is active.
            return true;
        } else if (time > 105) {
            // Shift 1
            return shift1Active;
        } else if (time > 105-BUFFER_TIME) {
            return true;
        } else if (time > 80) {
            // Shift 2
            return !shift1Active;
        } else if (time > 80-BUFFER_TIME) {
          return true;  
        } else if (time > 55) {
            // Shift 3
            return shift1Active;
        } else if (time > 55-BUFFER_TIME) {
            return true;
        } else if (time > 30) {
            // Shift 4
            return !shift1Active;
        } else {
            // End game, hub always active.
            return true;
        }
    }
}
