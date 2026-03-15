// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
//import frc.robot.commands.ShootOnTheMoveCommand;
import frc.robot.controls.DriverControls;
import frc.robot.controls.OperatorControls;
import frc.robot.controls.PoseControls;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.TurretSubsystem;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final IntakeSubsystem intake = new IntakeSubsystem();
    private final HopperSubsystem hopper = new HopperSubsystem();
    private final FeederSubsystem feeder = new FeederSubsystem();
    private final TurretSubsystem turret = new TurretSubsystem();
    private final ShooterSubsystem shooter = new ShooterSubsystem();
    private final ClimberSubsystem climber = new ClimberSubsystem();

    private final HoodSubsystem hood = new HoodSubsystem();

    private final Superstructure superstructure = new Superstructure(intake, hopper, feeder, turret, shooter, climber, hood);

    private final SendableChooser<Command> autoChooser;

    // track current alliance for change detection
    private Alliance currentAlliance = Alliance.Red;

    public RobotContainer() {
        configureBindings();
        buildNamedAutoCommands();

        if(!Robot.isReal() || true){
            DriverStation.silenceJoystickConnectionWarning(true);
        }

        // have autoChooser pull all PathPlanner autos as options
        autoChooser = new SendableChooser<>();

        // set default auto (do nothing)
        autoChooser.setDefaultOption("Do Nothing", Commands.none());

        // add a simple auto option to have the robot drive backward for 1 second then stop
        //autoChooser.addOption("Drive Backward", drivebase.driveBackwards().withTimeout(1));
        
        // put autoChooser on SmartDashboard
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);

        // set up controllers
        DriverControls.configure(Constants.ControllerConstants.kDriverControllerPort, superstructure);
        OperatorControls.configure(Constants.ControllerConstants.kOperatorControllerPort, superstructure);
        //PoseControls.configure(Constants.ControllerConstants.kPoseControllerPort);
  
    }

    public Command getAutonomousCommand() {
        /*// Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );*/
        return autoChooser.getSelected();
        // return new PathPlannerAuto("Basic Auto");
    }

    private Alliance getAlliance(){
        return DriverStation.getAlliance().orElse(Alliance.Red);
    }

    /*private boolean isInAllianceZone(){
        Alliance alliance = getAlliance();
        Distance blueZone = Inches.of(182);
        Distance redZone = Inches.of(469);

        if(alliance == Alliance.Blue && drivebase.getPose().getMeasureX().lt(blueZone)){
        return true;
        }else if(alliance == Alliance.Red && drivebase.getPose().getMeasureX().gt(redZone)){
        return true;
        }

        return false;
    }*/

    private void buildNamedAutoCommands(){
        // add any auto commands to NamedCommands here
        //NamedCommands.registerCommand("driveBackwards", drivebase.driveBackwards().withTimeout(1).withName("Auto.driveBackwards"));
        //NamedCommands.registerCommand("driveForwards", drivebase.driveForward().withTimeout(1).withName("Auto.driveForwards"));
    
        NamedCommands.registerCommand("aimShooting", superstructure.aimCommand(superstructure.getTargetShooterSpeed(), superstructure.getTargetTurretAngle(), superstructure.getTargetHoodAngle()).withName("Auto.AimCommand"));
        NamedCommands.registerCommand("stopShooting", superstructure.stopAllShootingCommand().withName("Auto.StopShooting"));
        NamedCommands.registerCommand("aimDynamicShooting", superstructure.aimDynamicCommand(() -> shooter.getSpeed(), () -> turret.getRawAngle(), () -> hood.getAngle()));
        
        NamedCommands.registerCommand("feedShooter", superstructure.feedAllCommand().withName("Auto.FeedShooter"));
        NamedCommands.registerCommand("stopFeed", superstructure.stopFeedingAllCommand().withName("Auto.StopFeed"));

        NamedCommands.registerCommand("climbUp", superstructure.moveClimberUp().withName("Auto.ClimbUp"));
        NamedCommands.registerCommand("climbDown", superstructure.moveClimberDown().withName("Auto.ClimbDown"));
        
        NamedCommands.registerCommand("deployIntake", superstructure.setIntakeDeployAndRoll().withName("Auto.DeployIntake"));
        NamedCommands.registerCommand("retractIntake", superstructure.setIntakeStow().withName("Auto.StowIntake"));
        NamedCommands.registerCommand("bounceIntake", superstructure.intakeBounceCommand().withName("Auto.BounceIntake"));
        NamedCommands.registerCommand("eject", superstructure.ejectAllCommand().withName("Auto.Eject"));

        NamedCommands.registerCommand("centerTurret", superstructure.setTurretForward().withName("Auto.CenterTurret"));
        NamedCommands.registerCommand("manualShoot", superstructure.shootCommand().withName("Auto.ManualShoot"));

        //NamedCommands.registerCommand("shootOnTheMove", new ShootOnTheMoveCommand(drivebase,superstructure,() -> superstructure.getAimPoint()).ignoringDisable(true).withName("Auto.Eject"));
    }
}
