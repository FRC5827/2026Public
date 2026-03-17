package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
import frc.robot.util.FieldUtil;
import frc.robot.util.GameTimeUtil;
import frc.robot.util.LoggedTunableNumber;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public final class Superstructure extends SubsystemBase {
    private final Shooter shooter;
    private final Turret turret;
    private final Kicker kicker;
    private final Indexer indexer;
    private final Targeting targeting;
    private final Drive drive;

    private final Field2d field2d = new Field2d();

    private static final LoggedTunableNumber hubEdgeDY =
            new LoggedTunableNumber("Targeting/Hub Edge dy", 0.6);
    private static final LoggedTunableNumber hubEdgeDX =
            new LoggedTunableNumber("Targeting/Hub Edge dx", 0.6);

    public static LoggedDashboardChooser<Boolean> overrideTimeRestrictions;

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
        this.drive = drive;

        overrideTimeRestrictions =
                new LoggedDashboardChooser<>("Override Time Restrictions", new SendableChooser<>());
        overrideTimeRestrictions.addDefaultOption("False", false);
        overrideTimeRestrictions.addOption("True", true);

        SmartDashboard.putData("Field2d", field2d);
    }

    @Override
    public void periodic() {
        double startTime = Timer.getFPGATimestamp();
        Logger.recordOutput(
                "Superstructure/Is Aiming At Hub",
                FieldUtil.isInCurrentAllianceZone(drive::getPose, DriverStation.getAlliance())
                        && targeting.canAimAtTarget());
        Logger.recordOutput(
                "PerformanceMonitor/Superstructure", (Timer.getFPGATimestamp() - startTime) * 1000);

        // Update Field2d
        field2d.setRobotPose(drive.getPose());
        if (targeting.hasTarget()) {
            field2d.getObject("Shooter Target")
                    .setPose(
                            new Pose2d(
                                    targeting.getTargetTranslation().toTranslation2d(),
                                    Rotation2d.kZero));
        } else {
            field2d.getObject("Shooter Target").setPose(new Pose2d(5, 5, Rotation2d.kZero));
        }
    }

    public Command aim() {
        return (Commands.either(
                aimAtHub(),
                aimAtCorner(),
                () ->
                        FieldUtil.isInCurrentAllianceZone(
                                drive::getPose, DriverStation.getAlliance())));
    }

    public Command aimAtHub() {
        return targeting
                .runOnce(
                        () -> {
                            targeting.setTarget(
                                    AllianceFlipUtil.shouldFlip()
                                            ? FieldConstants.Hub.oppInnerCenterPoint
                                            : FieldConstants.Hub.innerCenterPoint,
                                    new Translation2d(hubEdgeDX.get(), hubEdgeDY.get()));
                        })
                .andThen(turret.aimAtTarget())
                .finallyDo(() -> targeting.clearTarget());
    }

    public Command aimAtCorner() {
        return targeting
                .runOnce(() -> targeting.aimAtCorner())
                .andThen(turret.aimAtTarget())
                .finallyDo(() -> targeting.clearTarget());
    }

    public Command shootAtTarget() {
        return Commands.waitUntil(
                        () ->
                                turret.isAimingAtTarget()
                                        && (GameTimeUtil.isHubActive(
                                                        targeting.getAirTimeToTarget()
                                                                + DriverStation.getMatchTime())
                                                || overrideTimeRestrictions.get()))
                .andThen(shooter.shootAtTarget());
    }

    public Command runKickerAndIndexer() {
        return Commands.waitUntil(shooter::isShooterAtVelocity)
                .andThen(Commands.parallel(kicker.runKicker(), indexer.runIndexer()));
    }

    public Command aimAndShoot() {
        return Commands.parallel(aim(), shootAtTarget(), runKickerAndIndexer());
    }

    public boolean isShootingAtTarget() {
        return targeting.canAimAtTarget()
                && turret.isAimingAtTarget()
                && shooter.isShooterAtVelocity();
    }
}
