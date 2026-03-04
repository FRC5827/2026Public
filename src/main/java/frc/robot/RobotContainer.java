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
import edu.wpi.first.wpilibj.RobotBase;
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

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    private final Vision vision;
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

                vision =
                        new Vision(
                                drive::addVisionMeasurement,
                                new VisionIOPhotonVision(cameraFRName, robotToCameraFR),
                                new VisionIOPhotonVision(cameraFLName, robotToCameraFL),
                                new VisionIOPhotonVision(cameraBRName, robotToCameraBR),
                                new VisionIOPhotonVision(cameraBLName, robotToCameraBL));

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
                    vision =
                            new Vision(
                                    drive::addVisionMeasurement,
                                    new VisionIOPhotonVisionSim(
                                            cameraFRName, robotToCameraFR, drive::getPose),
                                    new VisionIOPhotonVisionSim(
                                            cameraFLName, robotToCameraFL, drive::getPose),
                                    new VisionIOPhotonVisionSim(
                                            cameraBRName, robotToCameraBR, drive::getPose),
                                    new VisionIOPhotonVisionSim(
                                            cameraBLName, robotToCameraBL, drive::getPose));
                } else {
                    vision =
                            new Vision(
                                    drive::addVisionMeasurement,
                                    new VisionIO() {},
                                    new VisionIO() {},
                                    new VisionIO() {},
                                    new VisionIO() {});
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

                vision =
                        new Vision(
                                drive::addVisionMeasurement,
                                new VisionIO() {},
                                new VisionIO() {},
                                new VisionIO() {},
                                new VisionIO() {});

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
        // used in an autos in PathPlanner.
        NamedCommands.registerCommand(
                "Shoot",
                RobotBase.isReal()
                        ? shooter.aimAtHub()
                                .alongWith(
                                        Commands.waitUntil(shooter::isAimedAtTarget)
                                                .andThen(shooter.shootAtTarget()))
                                .alongWith(
                                        Commands.waitUntil(shooter::shooterRunningAtVelocity)
                                                .andThen(
                                                        hopperIndexer
                                                                .runIndexer()
                                                                .alongWith(
                                                                        hopperKicker.runKicker())))
                        : Commands.waitSeconds(5.0));
        // Yes I know they're the same thing, but we need to tune the timeout and they'll be
        // different afterwards probably, as the timeouts for real and sim are for different
        // purposes
        NamedCommands.registerCommand("Intake", intakeFlywheel.runIntake().withTimeout(2.0));
        // We don't have a climber yet
        NamedCommands.registerCommand("Climb", Commands.print("CLIMB!"));
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
                        () -> -controller.getRightX()));

        // Lock to 0° when A button is held
        controller
                .a()
                .whileTrue(
                        DriveCommands.joystickDriveAtAngle(
                                drive,
                                () -> -controller.getLeftY(),
                                () -> -controller.getLeftX(),
                                () -> Rotation2d.kZero));

        // Switch to X pattern when X button is pressed
        controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

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
                                .ignoringDisable(true));

        // intake controls
        controller.leftTrigger().whileTrue(intakeFlywheel.runIntake());

        controller.povUp().whileTrue(intakeFlywheel.runReverse());

        // shoot controls
        turret.setDefaultCommand(superstructure.aimAtHub());
        controller.rightTrigger().whileTrue(superstructure.aimAndShoot());

        // Hopper and Kicker controls
        controller
                .povRight()
                .whileTrue(hopperIndexer.runIndexerReverse())
                .whileTrue(hopperKicker.runKickerReverse());

        controller
                .povDown()
                .onTrue(intakeDeployer.retractDeployer())
                .onFalse(intakeDeployer.deployDeployer());

        controller.rightBumper().whileTrue(intakeDeployer.deployerUp());
        controller.leftBumper().whileTrue(intakeDeployer.deployerDown());

        // temporary testing command for tuning shooter
        controller
                .rightStick()
                .whileTrue(
                        targeting
                                .runOnce(() -> targeting.setTargetManual())
                                .finallyDo(() -> targeting.clearTarget())
                                .alongWith(turret.aimAtTarget())
                                .alongWith(superstructure.shootAtTarget())
                                .alongWith(superstructure.runKickerAndIndexer()));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    /**
     * Checks if the robot is aligned with either the left or right trench opening. Call this
     * periodically (e.g., in Robot.java) to detect alignment changes.
     */
    public void checkTrenchAlignment() {
        Pose2d robotPose = drive.getPose();
        boolean isCurrentlyAligned = isTrenchAligned(robotPose);

        // Only rumble when newly aligned (transition from not aligned to aligned)
        if (isCurrentlyAligned && !wasPreviouslyAligned) {
            controller.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.2);
        } else if (!isCurrentlyAligned && wasPreviouslyAligned) {
            // Stop rumble when no longer aligned
            controller.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
        }

        wasPreviouslyAligned = isCurrentlyAligned;
    }

    /**
     * Determines if the robot is aligned with the trench opening. This checks if the entire robot
     * is within the left or right trench opening bounds, not just the center point. Also excludes
     * corner areas (near the driver stations) from triggering rumble.
     *
     * @param robotPose the robot's current pose
     * @return true if the entire robot is aligned with a trench opening and not near driver
     *     stations
     */
    private boolean isTrenchAligned(Pose2d robotPose) {
        double robotY = robotPose.getY();
        double robotX = robotPose.getX();
        // Robot bumper to bumper length (Y dimension) - 0.826m is width, kept for reference
        double halfLength = Constants.ROBOT_LENGTH / 2.0;

        // Calculate the bounds of the entire robot
        double robotYMin = robotY - halfLength;
        double robotYMax = robotY + halfLength;

        // Check if entire robot fits within left trench opening
        boolean isAlignedWithLeftTrench =
                robotYMin >= FieldConstants.LinesHorizontal.leftTrenchOpenEnd
                        && robotYMax <= FieldConstants.LinesHorizontal.leftTrenchOpenStart;

        // Check if entire robot fits within right trench opening
        boolean isAlignedWithRightTrench =
                robotYMin >= FieldConstants.LinesHorizontal.rightTrenchOpenEnd
                        && robotYMax <= FieldConstants.LinesHorizontal.rightTrenchOpenStart;

        // Exclude corners near driver stations - don't rumble near the ends of the field
        // Field length is approximately 16.54 meters, so corners are at X near 0 and X near 16.54
        double cornerExclusionDistance = 1.2; // meters from each end
        boolean isNearDriverStations =
                robotX < cornerExclusionDistance
                        || robotX > (FieldConstants.fieldLength - cornerExclusionDistance);

        boolean isTrenchAligned =
                (isAlignedWithLeftTrench || isAlignedWithRightTrench) && !isNearDriverStations;

        return isTrenchAligned;
    }
}
