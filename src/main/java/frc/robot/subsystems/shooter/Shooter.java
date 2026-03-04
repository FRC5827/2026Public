package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.util.LoggedTunableNumber;

import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    static final double SHOOTER_GEAR_RATIO = 1.0; // sensor to mechanism ratio
    static final double SHOOTER_RADIUS_METERS = Units.inchesToMeters(2.0);

    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs;

    private final Targeting targeting;

    private static final LoggedTunableNumber shooterOpenLoopVoltage =
            new LoggedTunableNumber("Shooter/Shooter/Open Loop Voltage", 0.5);
    private static final LoggedTunableNumber shooterTolerance =
            new LoggedTunableNumber("Shooter/Shooter/ToleranceMPS", 0.1);
    static final LoggedTunableNumber shooterKP = new LoggedTunableNumber("Shooter/Shooter/kP", 0.5);
    static final LoggedTunableNumber shooterKD = new LoggedTunableNumber("Shooter/Shooter/kD", 0.0);
    static final LoggedTunableNumber shooterKS = new LoggedTunableNumber("Shooter/Shooter/kS", 0.0);
    static final LoggedTunableNumber shooterKV =
            new LoggedTunableNumber("Shooter/Shooter/kV", 0.76);

    private SimpleMotorFeedforward shooterFeedforward =
            new SimpleMotorFeedforward(shooterKS.get(), shooterKV.get());

    public Shooter(ShooterIO io, Targeting targeting) {
        this.io = io;
        this.inputs = new ShooterIOInputsAutoLogged();
        this.targeting = targeting;
    }

    @Override
    public void periodic() {
        double startTime = Timer.getFPGATimestamp();
        io.updateInputs(inputs);

        checkForPIDUpdates();

        Logger.processInputs("Shooter", inputs);
        Logger.recordOutput(
                "PerformanceMonitor/Shooter", (Timer.getFPGATimestamp() - startTime) * 1000);
    }

    public Command shoot() {
        return this.startEnd(
                () -> {
                    io.setShooterVoltage(shooterOpenLoopVoltage.get());
                },
                () -> {
                    io.setShooterVoltage(0);
                });
    }

    public Command shootAtTarget() {
        return this.runEnd(
                () -> {
                    double targetVelocity = targeting.getShooterVelocity();
                    io.setShooterVelocity(
                            targetVelocity, shooterFeedforward.calculate(targetVelocity));
                },
                () -> {
                    io.setShooterVoltage(0);
                });
    }

    public void setShooterVoltage(double voltage) {
        io.setShooterVoltage(voltage);
    }

    private void checkForPIDUpdates() {
        if (shooterKP.hasChanged(this.hashCode()) || shooterKD.hasChanged(this.hashCode())) {
            io.updateShooterPID(shooterKP.get(), shooterKD.get());
        }

        if (shooterKS.hasChanged(this.hashCode()) || shooterKV.hasChanged(this.hashCode())) {
            shooterFeedforward.setKs(shooterKS.get());
            shooterFeedforward.setKv(shooterKV.get());
        }
    }

    public boolean isShooterAtVelocity() {
        return targeting.canAimAtTarget()
                && MathUtil.isNear(
                        targeting.getShooterVelocity(),
                        inputs.shooterMotorVelocityMPS,
                        shooterTolerance.get());
    }

    public double getShooterVelocity() {
        return inputs.shooterMotorVelocityMPS;
    }
}
