package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hopper.indexer.Indexer;
import frc.robot.subsystems.hopper.kicker.Kicker;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Targeting;
import frc.robot.subsystems.shooter.Turret;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.GameTimeUtil;
import frc.robot.util.LoggedTunableNumber;

import org.littletonrobotics.junction.Logger;

public final class Superstructure extends SubsystemBase {
    private final Shooter shooter;
    private final Turret turret;
    private final Kicker kicker;
    private final Indexer indexer;
    private final Targeting targeting;

    private static final LoggedTunableNumber hubEdgeDY =
            new LoggedTunableNumber("Shooter/Hub Edge dy", 0.3);
    private static final LoggedTunableNumber hubEdgeDX =
            new LoggedTunableNumber("Shooter/Hub Edge dx", 0.84);

    public Superstructure(
            Shooter shooter,
            Targeting targeting,
            Turret turret,
            Kicker kicker,
            Indexer indexer,
            Drive drive) {
        this.shooter = shooter;
        this.turret = turret;
        this.kicker = kicker;
        this.indexer = indexer;
        this.targeting = targeting;
    }

    @Override
    public void periodic() {
        double startTime = Timer.getFPGATimestamp();
        Logger.recordOutput(
                "PerformanceMonitor/Superstructure", (Timer.getFPGATimestamp() - startTime) * 1000);
    }

    public Command aimAtHub() {
        return targeting
                .runOnce(
                        () -> {
                            double airTime = targeting.getAirTimeToTarget();
                            if (GameTimeUtil.isHubActive(airTime + DriverStation.getMatchTime())
                                    && !Targeting.overrideTimeRestrictions.get()) {
                                targeting.setTarget(
                                        AllianceFlipUtil.shouldFlip()
                                                ? FieldConstants.Hub.oppInnerCenterPoint
                                                : FieldConstants.Hub.innerCenterPoint,
                                        new Translation2d(hubEdgeDX.get(), hubEdgeDY.get()));
                            } else {
                                targeting.clearTarget();
                            }
                        })
                .andThen(turret.aimAtTarget())
                .finallyDo(() -> targeting.clearTarget());
    }

    public Command aimAtCorner() {
        return targeting.runOnce(() -> targeting.aimAtCorner());
    }

    public Command shootAtTarget() {
        return Commands.waitUntil(turret::isAimingAtTarget).andThen(shooter.shootAtTarget());
    }

    public Command runKickerAndIndexer() {
        return Commands.waitUntil(shooter::isShooterAtVelocity)
                .andThen(Commands.parallel(kicker.runKicker(), indexer.runIndexer()));
    }

    public Command aimAndShoot() {
        return Commands.parallel(aimAtHub(), shootAtTarget(), runKickerAndIndexer());
    }

    public boolean isShootingAtTarget() {
        return targeting.canAimAtTarget()
                && turret.isAimingAtTarget()
                && shooter.isShooterAtVelocity();
    }
}
