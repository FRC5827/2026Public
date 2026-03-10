package frc.robot.subsystems.intake.deployer;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.util.LoggedTunableNumber;

import org.littletonrobotics.junction.Logger;

public class Deployer extends SubsystemBase {

    static final double DEPLOYER_POSITION_TOLERANCE_RAD = Units.degreesToRadians(5);
    static final double DEPLOYER_VELOCITY_TOLERANCE_RAD_PER_SEC = Units.degreesToRadians(5);

    // PID constants - subject to change
    // for motion profiling
    static final LoggedTunableNumber DEPLOYER_kP =
            new LoggedTunableNumber("Intake/Deployer/kP", 30);
    static final LoggedTunableNumber DEPLOYER_kD = new LoggedTunableNumber("Intake/Deployer/kD", 0);

    // feedforward constants
    static final LoggedTunableNumber DEPLOYER_kS =
            new LoggedTunableNumber("Intake/Deployer/kS", 0.0);
    static final LoggedTunableNumber DEPLOYER_kV =
            new LoggedTunableNumber("Intake/Deployer/kV", 0.0);
    static final LoggedTunableNumber DEPLOYER_kA =
            new LoggedTunableNumber("Intake/Deployer/kA", 0.0);
    static final LoggedTunableNumber DEPLOYER_kG =
            new LoggedTunableNumber("Intake/Deployer/kG", 0.0);

    static final LoggedTunableNumber DEPLOYER_MAX_VELOCITY =
            new LoggedTunableNumber("Intake/Depoloyer/Max Velocity", 3.0);
    static final LoggedTunableNumber DEPLOYER_MAX_ACCELERATION =
            new LoggedTunableNumber("Intake/Deployer/Max Acceleration", 3.0);

    // target points, 0 is always horizontal by convention
    static final LoggedTunableNumber DEPLOYER_RETRACT_ANGLE_RAD =
            new LoggedTunableNumber(
                    "Intake/DeployerRetractPosition",
                    Units.rotationsToRadians(0.385254)); // subject to change
    static final LoggedTunableNumber DEPLOYER_DEPLOY_ANGLE_RAD =
            new LoggedTunableNumber(
                    "Intake/Deployer/DeployPosition",
                    Units.rotationsToRadians(0.030518)); // subject to change
    private final LoggedTunableNumber deployerVoltage =
            new LoggedTunableNumber("Intake/Deployer/Voltage", 2.0);

    // deployer max/min angles
    static final Angle DEPLOYER_MAX_ANGLE =
            Radians.of(DEPLOYER_RETRACT_ANGLE_RAD.get()); // subject to change
    static final Angle DEPLOYER_MIN_ANGLE =
            Radians.of(DEPLOYER_DEPLOY_ANGLE_RAD.get()); // subject to change

    static final double DEPLOYER_GEAR_RATIO = 10;

    private final DeployerIO io;
    private final DeployerIOInputsAutoLogged inputs;
    private final ArmFeedforward deployerFeedforward;
    private TrapezoidProfile motionProfile;
    private TrapezoidProfile.State profileCurrentState, profileGoalState;
    private boolean doMotionProfiling = false;
    private boolean atSetpoint = false;

    public Deployer(DeployerIO io) {
        this.io = io;
        this.inputs = new DeployerIOInputsAutoLogged();

        deployerFeedforward =
                new ArmFeedforward(
                        DEPLOYER_kS.get(), DEPLOYER_kG.get(), DEPLOYER_kV.get(), DEPLOYER_kA.get());
        motionProfile =
                new TrapezoidProfile(
                        new TrapezoidProfile.Constraints(
                                DEPLOYER_MAX_VELOCITY.get(), DEPLOYER_MAX_ACCELERATION.get()));

        profileCurrentState = new TrapezoidProfile.State(DEPLOYER_RETRACT_ANGLE_RAD.get(), 0.0);
        profileGoalState = new TrapezoidProfile.State(DEPLOYER_RETRACT_ANGLE_RAD.get(), 0.0);
    }

    @Override
    public void periodic() {
        double startTime = Timer.getFPGATimestamp();
        io.updateInputs(inputs);
        Logger.processInputs("Intake/Deployer", inputs);

        Logger.recordOutput(
                "Intake/Deployer/Current Profile Position", profileCurrentState.position);
        Logger.recordOutput(
                "Intake/Deployer/Current Profile Velocity", profileCurrentState.velocity);

        if (doMotionProfiling) {
            var profileNewState =
                    motionProfile.calculate(0.02, profileCurrentState, profileGoalState);
            double feedforwardVoltage =
                    deployerFeedforward.calculateWithVelocities(
                            profileCurrentState.position,
                            profileCurrentState.velocity,
                            profileNewState.velocity);
            io.setDeployerState(
                    profileCurrentState.position, profileCurrentState.velocity, feedforwardVoltage);
            profileCurrentState = profileNewState;
        }

        atSetpoint =
                MathUtil.isNear(
                                profileGoalState.position,
                                inputs.deployerPositionRadians,
                                DEPLOYER_POSITION_TOLERANCE_RAD)
                        && MathUtil.isNear(
                                profileGoalState.velocity,
                                inputs.deployerVelocityRadPerSec,
                                DEPLOYER_VELOCITY_TOLERANCE_RAD_PER_SEC);
        Logger.recordOutput("Intake/Deployer/At Setpoint", atSetpoint);

        if (DEPLOYER_kP.hasChanged(this.hashCode()) || DEPLOYER_kD.hasChanged(this.hashCode())) {
            io.updatePID(DEPLOYER_kP.get(), DEPLOYER_kD.get());
        }

        if (DEPLOYER_MAX_VELOCITY.hasChanged(this.hashCode())
                || DEPLOYER_MAX_ACCELERATION.hasChanged(this.hashCode())) {
            motionProfile =
                    new TrapezoidProfile(
                            new TrapezoidProfile.Constraints(
                                    DEPLOYER_MAX_VELOCITY.get(), DEPLOYER_MAX_ACCELERATION.get()));
        }

        if (DEPLOYER_kS.hasChanged(this.hashCode())
                || DEPLOYER_kV.hasChanged(this.hashCode())
                || DEPLOYER_kA.hasChanged(this.hashCode())
                || DEPLOYER_kG.hasChanged(this.hashCode())) {
            deployerFeedforward.setKs(DEPLOYER_kS.get());
            deployerFeedforward.setKv(DEPLOYER_kV.get());
            deployerFeedforward.setKa(DEPLOYER_kA.get());
            deployerFeedforward.setKg(DEPLOYER_kG.get());
        }
        Logger.recordOutput(
                "PerformanceMonitor/Intake/Deployer",
                (Timer.getFPGATimestamp() - startTime) * 1000);
    }

    public void updateProfileStates() {
        profileCurrentState.position = inputs.deployerPositionRadians;
        profileCurrentState.velocity = inputs.deployerVelocityRadPerSec;
    }

    public void setBrakeMode(boolean brake) {
        io.setDeployerBrakeMode(brake);
    }

    public Command stopDeployer() {
        return this.run(
                () -> {
                    io.setDeployerVoltage(0);
                    doMotionProfiling = false;
                });
    }

    // Open Loop Control
    public Command deployerUp() {
        return this.startEnd(
                () -> {
                    io.setDeployerVoltage(deployerVoltage.getAsDouble());
                    doMotionProfiling = false;
                },
                () -> io.setDeployerVoltage(0));
    }

    public Command deployerDown() {
        return this.startEnd(
                () -> {
                    io.setDeployerVoltage(-deployerVoltage.getAsDouble());
                    doMotionProfiling = false;
                },
                () -> io.setDeployerVoltage(0));
    }

    // Closed Loop control
    public Command retractDeployer() {
        return this.runOnce(
                        () -> {
                            profileGoalState.position = DEPLOYER_RETRACT_ANGLE_RAD.get();
                            atSetpoint = false;
                            doMotionProfiling = true;
                        })
                .andThen(Commands.waitUntil(() -> atSetpoint));
    }

    public Command deployDeployer() {
        return this.runOnce(
                        () -> {
                            profileGoalState.position = DEPLOYER_DEPLOY_ANGLE_RAD.get();
                            atSetpoint = false;
                            doMotionProfiling = true;
                        })
                .andThen(Commands.waitUntil(() -> atSetpoint));
    }
}
