// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.FieldConstants.AprilTagLayoutType;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.autos.AutoChooser;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.hopper.indexer.Indexer;
import frc.robot.subsystems.hopper.indexer.IndexerIO;
import frc.robot.subsystems.hopper.indexer.IndexerIOSim;
import frc.robot.subsystems.hopper.indexer.IndexerIOTalonFX;
import frc.robot.subsystems.hopper.kicker.Kicker;
import frc.robot.subsystems.hopper.kicker.KickerIO;
import frc.robot.subsystems.hopper.kicker.KickerIOSim;
import frc.robot.subsystems.hopper.kicker.KickerIOTalonFX;
import frc.robot.subsystems.intake.deployer.Deployer;
import frc.robot.subsystems.intake.deployer.DeployerIO;
import frc.robot.subsystems.intake.deployer.DeployerIOSim;
import frc.robot.subsystems.intake.deployer.DeployerIOTalonFX;
import frc.robot.subsystems.intake.flywheel.Flywheel;
import frc.robot.subsystems.intake.flywheel.FlywheelIO;
import frc.robot.subsystems.intake.flywheel.FlywheelIOSim;
import frc.robot.subsystems.intake.flywheel.FlywheelIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOReal;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.Targeting;
import frc.robot.subsystems.shooter.Turret;
import frc.robot.subsystems.shooter.TurretIO;
import frc.robot.subsystems.shooter.TurretIOReal;
import frc.robot.subsystems.shooter.TurretIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.FieldUtil;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    private final Drive drive;
    private final Flywheel intakeFlywheel;
    private final Targeting targeting;
    private final Shooter shooter;
    private final Deployer intakeDeployer;
    private final Indexer hopperIndexer;
    private final Kicker hopperKicker;
    private final Turret turret;
    private final Superstructure superstructure;

    // Controller
    private final CommandXboxController controller = new CommandXboxController(0);

    private AutoChooser autoChooser;

    // Trench alignment detection
    private boolean wasPreviouslyAligned = false;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // initialize all AprilTag field layouts at start to avoid delays when first getting them
        for (AprilTagLayoutType type : AprilTagLayoutType.values()) {
            type.getLayout();
        }

        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
                // a CANcoder
                drive =
                        new Drive(
                                new GyroIOPigeon2(),
                                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                                new ModuleIOTalonFX(TunerConstants.FrontRight),
                                new ModuleIOTalonFX(TunerConstants.BackLeft),
                                new ModuleIOTalonFX(TunerConstants.BackRight));

                // Initialize vision (local scope only, not stored)
                new Vision(
                        drive::addVisionMeasurement,
                        new VisionIOPhotonVision(cameraFRName, robotToCameraFR),
                        new VisionIOPhotonVision(cameraFLName, robotToCameraFL)
                        // new VisionIOPhotonVision(cameraBRName, robotToCameraBR),
                        // new VisionIOPhotonVision(cameraBLName, robotToCameraBL)
                        );

                intakeDeployer = new Deployer(new DeployerIOTalonFX());

                intakeFlywheel = new Flywheel(new FlywheelIOTalonFX());

                targeting = new Targeting(drive::getPose);
                shooter = new Shooter(new ShooterIOReal(), targeting);
                turret = new Turret(new TurretIOReal(), targeting);

                hopperIndexer = new Indexer(new IndexerIOTalonFX());
                hopperKicker = new Kicker(new KickerIOTalonFX());

                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                drive =
                        new Drive(
                                new GyroIO() {},
                                new ModuleIOSim(TunerConstants.FrontLeft),
                                new ModuleIOSim(TunerConstants.FrontRight),
                                new ModuleIOSim(TunerConstants.BackLeft),
                                new ModuleIOSim(TunerConstants.BackRight));

                if (Constants.simWithVision) {
                    // Initialize vision (local scope only, not stored)
                    new Vision(
                            drive::addVisionMeasurement,
                            new VisionIOPhotonVisionSim(
                                    cameraFRName, robotToCameraFR, drive::getPose),
                            new VisionIOPhotonVisionSim(
                                    cameraFLName, robotToCameraFL, drive::getPose)
                            // new VisionIOPhotonVisionSim(
                            //         cameraBRName, robotToCameraBR, drive::getPose),
                            // new VisionIOPhotonVisionSim(
                            //         cameraBLName, robotToCameraBL, drive::getPose)
                            );
                } else {
                    // Initialize vision (local scope only, not stored)
                    new Vision(
                            drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {}
                            // new VisionIO() {},
                            // new VisionIO() {}
                            );
                }

                intakeDeployer = new Deployer(new DeployerIOSim());
                intakeFlywheel = new Flywheel(new FlywheelIOSim());

                targeting = new Targeting(drive::getPose);

                shooter = new Shooter(new ShooterIOSim(), targeting);
                turret = new Turret(new TurretIOSim(), targeting);

                hopperIndexer = new Indexer(new IndexerIOSim());
                hopperKicker = new Kicker(new KickerIOSim());
                break;

            default:
                drive =
                        new Drive(
                                new GyroIO() {},
                                new ModuleIO() {},
                                new ModuleIO() {},
                                new ModuleIO() {},
                                new ModuleIO() {});

                // Initialize vision (local scope only, not stored)
                new Vision(
                        drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {}
                        // new VisionIO() {},
                        // new VisionIO() {}
                        );

                intakeDeployer = new Deployer(new DeployerIO() {});
                intakeFlywheel = new Flywheel(new FlywheelIO() {});

                targeting = new Targeting(drive::getPose);
                shooter = new Shooter(new ShooterIO() {}, targeting);
                turret = new Turret(new TurretIO() {}, targeting);

                hopperIndexer = new Indexer(new IndexerIO() {});
                hopperKicker = new Kicker(new KickerIO() {});

                break;
        }

        // Initialize superstructure
        superstructure =
                new Superstructure(shooter, targeting, turret, hopperKicker, hopperIndexer, drive);

        // Set up auto chooser
        autoChooser = new AutoChooser(drive);

        registerNamedCommands();

        // Configure the button bindings
        configureButtonBindings();
    }

    private void registerNamedCommands() {
        // Named commands are commands in PathPlanner that are given a name so they can be directly
        // used in autos in PathPlanner. These should use only direct subsystem commands, not
        // Superstructure (which is reserved for button bindings and manual control).

        // Shoot command: aim at hub, wait for shooter to reach velocity, then shoot with hopper.
        // Wrapped in asProxy() so the auto sequence doesn't hold Turret/Targeting requirements
        // for the entire auto — this allows the turret's default aiming command to run between
        // path segments.
        // I don't like proxying, but unfortunately it's the only way that works without massive
        // refactoring
        NamedCommands.registerCommand("Shoot", superstructure.aimAndShoot().asProxy());

        // Intake command: run intake flywheel (proxied so auto doesn't hold Flywheel requirement)
        NamedCommands.registerCommand(
                "Intake",
                intakeFlywheel.runIntake().withTimeout(2.0).withName("NC_Intake").asProxy());

        // Climb command: placeholder for future implementation
        NamedCommands.registerCommand("Climb", Commands.print("CLIMB!").withName("NC_Climb"));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Default command, normal field-relative drive
        drive.setDefaultCommand(
                DriveCommands.joystickDrive(
                                drive,
                                () -> -controller.getLeftY(),
                                () -> -controller.getLeftX(),
                                () -> -controller.getRightX())
                        .withName("Drive_JoystickDrive_Default"));

        // Lock to 0° when A button is held
        controller
                .a()
                .whileTrue(
                        DriveCommands.joystickDriveAtAngle(
                                        drive,
                                        () -> -controller.getLeftY(),
                                        () -> -controller.getLeftX(),
                                        () -> Rotation2d.kZero)
                                .withName("Drive_LockTo0Deg_A"));

        // Reset gyro to 0° when B button is pressed
        controller
                .b()
                .onTrue(
                        Commands.runOnce(
                                        () ->
                                                drive.setPose(
                                                        new Pose2d(
                                                                drive.getPose().getTranslation(),
                                                                Rotation2d.kZero)),
                                        drive)
                                .ignoringDisable(true)
                                .withName("Drive_ResetGyro_B"));

        // intake controls
        controller
                .leftTrigger()
                .whileTrue(intakeFlywheel.runIntake().withName("Intake_RunFlywheel_LT"));
        controller
                .leftBumper()
                .whileTrue(intakeFlywheel.runIntake().withName("Intake_RunFlywheel_LB"))
                .onTrue(intakeDeployer.liftDeployer().withName("Deployer_Lift_LB_OnTrue"))
                .onFalse(intakeDeployer.deployDeployer().withName("Deployer_Deploy_LB_OnFalse"));

        controller
                .povUp()
                .whileTrue(intakeFlywheel.runReverse().withName("Intake_RunReverse_PovUp"));

        // shoot controls
        turret.setDefaultCommand(superstructure.aim().withName("Turret_Aim_Default"));
        controller
                .rightTrigger()
                .whileTrue(superstructure.aimAndShoot().withName("Superstructure_AimAndShoot_RT"))
                .onTrue(DriveCommands.setSlowMode(true).withName("Set_Slow_Mode"))
                .onFalse(DriveCommands.setSlowMode(false).withName("Set_Slow_Mode"));

        // Hopper and Kicker controls
        controller
                .povRight()
                .whileTrue(
                        hopperIndexer.runIndexerReverse().withName("Indexer_RunReverse_PovRight"))
                .whileTrue(hopperKicker.runKickerReverse().withName("Kicker_RunReverse_PovRight"));

        controller.povLeft().whileTrue(hopperIndexer.runIndexer().withName("Indexer_Run_PovLeft"));

        controller
                .povDown()
                .onTrue(
                        intakeDeployer
                                .retractDeployer()
                                .withName("Deployer_Retract_PovDown_OnTrue"))
                .onFalse(
                        intakeDeployer
                                .deployDeployer()
                                .withName("Deployer_Deploy_PovDown_OnFalse"));

        // temporary testing command for tuning shooter
        controller
                .rightStick()
                .whileTrue(
                        Commands.run(() -> targeting.setTargetManual())
                                .withName("Targeting_SetManual")
                                .alongWith(
                                        turret.aimAtTarget()
                                                .withName("Turret_AimAtTarget_RS")
                                                .alongWith(
                                                        superstructure
                                                                .shootAtTarget()
                                                                .withName(
                                                                        "Superstructure_ShootAtTarget_RS"))
                                                .alongWith(
                                                        superstructure
                                                                .runKickerAndIndexer()
                                                                .withName(
                                                                        "Superstructure_RunKickerAndIndexer_RS")))
                                .finallyDo(() -> targeting.clearTarget())
                                .withName("ManualShootTest_RightStick"));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // return Commands.parallel(intakeDeployer.liftDeployer(), new PathPlannerAuto("test"));
        return Commands.parallel(
                        intakeDeployer.liftDeployer().withName("Auto_DeployIntake"),
                        autoChooser.get())
                .withName("Auto_FullSequence");
    }

    /**
     * Checks if the robot is aligned with either the left or right trench opening. Call this
     * periodically (e.g., in Robot.java) to detect alignment changes.
     */
    public void checkTrenchAlignment() {
        boolean isCurrentlyAligned = FieldUtil.isTrenchAligned(drive::getPose);

        // Only rumble when newly aligned (transition from not aligned to aligned)
        if (isCurrentlyAligned && !wasPreviouslyAligned) {
            controller.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.2);
        } else if (!isCurrentlyAligned && wasPreviouslyAligned) {
            // Stop rumble when no longer aligned
            controller.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
        }

        if (isCurrentlyAligned) {
            targeting.lowerForTrench();
        }

        wasPreviouslyAligned = isCurrentlyAligned;
    }

    public void enable() {
        turret.setBrakeMode(true);
        intakeDeployer.setBrakeMode(true);
    }

    public void disable() {
        turret.setBrakeMode(false);
        intakeDeployer.setBrakeMode(false);
    }
}
