package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;

public final class Superstructure extends SubsystemBase {
    private final Shooter shooter;
    private final Drive drive;

    public Superstructure(Shooter shooter, Drive drive) {
        this.shooter = shooter;
        this.drive = drive;
    }
}
