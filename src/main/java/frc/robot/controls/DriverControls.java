package frc.robot.controls;

import static edu.wpi.first.units.Units.Meter;

/*import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.littletonrobotics.junction.Logger;*/

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.Constants.ControllerConstants;
//import frc.robot.commands.ShootOnTheMoveCommand;
import frc.robot.subsystems.Superstructure;
/*import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.maplesim.RebuiltFuelOnFly;
import swervelib.SwerveInputStream;*/

import frc.robot.Constants;

public class DriverControls {
    private static Pose2d getTargetPose(){
        Pose2d hubPose = new Pose2d(
            Meter.of(11.902),
            Meter.of(4.031),
            Rotation2d.kZero);
        
        //Logger.recordOutput("DriverControls/TargetHubPose", hubPose);

        return hubPose;
    }

    public static void configure(int port, Superstructure superstructure){
        CommandXboxController controller = new CommandXboxController(port);

        /*SwerveInputStream driveInputStream = SwerveInputStream.of(drivetrain.getSwerveDrive(),
            () -> controller.getLeftY() * -1,
            () -> controller.getLeftX() * -1)
            .withControllerRotationAxis(() -> controller.getRightX() * -1)
            .robotRelative(false)
            .allianceRelativeControl(true)
            .scaleTranslation(0.25) // TODO: tune speed scaling
            .deadband(ControllerConstants.DEADBAND);
        
        controller.rightBumper().whileTrue(Commands.run(
            () -> {
                driveInputStream
                .aim(getTargetPose())
                .aimWhile(true);
            }).finallyDo(() -> driveInputStream.aimWhile(false)));
        
        drivetrain.setDefaultCommand(
            drivetrain.driveFieldOriented(driveInputStream).withName("Drive" + ".test")
        );*/

        // YAGSL way of doing direct drive to pose
        // driveInputStream.driveToPose(drivetrain.getTargetPoseSupplier(),
        // new ProfiledPIDController(5, 0, 0,
        // new Constraints(5, 2)),
        // new ProfiledPIDController(5, 0, 0,
        // new Constraints(
        // Units.degreesToRadians(360),
        // Units.degreesToRadians(180))));

        // controller.rightBumper().whileTrue(Commands.runEnd(
        // () -> driveInputStream.driveToPoseEnabled(true),
        // () -> driveInputStream.driveToPoseEnabled(false)));

        // pathplanner way of doing obstacle-aware drive to pose
        // controller.rightBumper()
        // .whileTrue(Commands.defer(
        // () -> drivetrain.driveToPose(drivetrain.getTargetPose()),
        // java.util.Set.of(drivetrain)));

        if(DriverStation.isTest()){
            // drivetrain.setDefaultCommand(driveFieldorientedAngularVelocity)
            // overrides drive command above
            // might be useful for robot-oriented controls in testing

            /*controller.povDown().whileTrue(drivetrain.centerModulesCommand());
            controller.povLeft().whileTrue(Commands.runOnce(drivetrain::lock, drivetrain).repeatedly());
            controller.povRight().onTrue(Commands.runOnce(drivetrain::zeroGyro));

            controller.start().whileTrue(drivetrain.sysIdAngleMotorCommand());
            controller.back().whileTrue(drivetrain.sysIdDriveMotorCommand());*/
        }else if(Robot.isSimulation()){
            // fire fuel 10 times per second while button is held
            /*controller.rightTrigger().whileTrue(
                Commands.repeatingSequence(
                    fireFuel(drivetrain, superstructure),
                    Commands.waitSeconds(0.1)
                )
            );*/
        }else{
            //controller.start().onTrue(superstructure.rezeroIntakePivotAndTurretCommand().ignoringDisable(true));

            controller.leftTrigger().whileTrue(superstructure.setIntakeDeployAndRoll().finallyDo(() -> superstructure.stopIntakeCommand().schedule()).withName("DriverControls.DeployIntake"));
            //controller.leftBumper().whileTrue(superstructure.intakeCommand().finallyDo(() -> superstructure.stopIntakeCommand().schedule()).withName("DriverControls.DeployIntake"));
            controller.rightBumper().whileTrue(superstructure.deployIntakeCommand().finallyDo(() -> superstructure.stopIntakePivot().schedule()).withName("DriverControls.Deploy"));
            controller.b().whileTrue(superstructure.returnIntakeCommand().finallyDo(() -> superstructure.stopIntakePivot().schedule()));
            
            /*controller.rightBumper().toggleOnTrue(new ShootOnTheMoveCommand(drivetrain, superstructure, () -> superstructure.getAimPoint())
                .ignoringDisable(true)
                .withName("DriverControls.ShootOnTheMove"));
            */
            controller.y().whileTrue(superstructure.feederFeedCommand());
            controller.x().whileTrue(superstructure.shootCommand().finallyDo(() -> superstructure.stopAllShootingCommand().schedule()));
            controller.a().whileTrue(superstructure.shootSlowCommand().finallyDo(() -> superstructure.stopAllShootingCommand().schedule()));

            //controller.leftTrigger().whileTrue(superstructure.aimCommand(superstructure.getTargetShooterSpeed(), superstructure.getTargetTurretAngle(), superstructure.getTargetHoodAngle()).finallyDo(() -> superstructure.stopAllShootingCommand().schedule()));

            controller.rightTrigger().whileTrue(superstructure.feedAllCommand().finallyDo(() -> superstructure.stopFeedingAllCommand().schedule()));

            controller.povUp().onTrue(superstructure.setTurretForward().finallyDo(() -> superstructure.stopTurret().schedule()).withName("DriverControls.SetTurretForward"));
            controller.povLeft().whileTrue(superstructure.rotateTurretLeft().finallyDo(() -> superstructure.stopTurret().schedule()));
            controller.povRight().whileTrue(superstructure.rotateTurretRight().finallyDo(() -> superstructure.stopTurret().schedule()));

            
            //controller.povDown().onTrue(Commands.runOnce(drivetrain::zeroGyro));

            //controller.y().whileTrue(superstructure.moveClimberDown());
            //controller.a().whileTrue(superstructure.moveClimberUp());
            // TODO: code in the vision for auto-targeting to tower (button x)
        }
    }

    /*public static Command fireFuel(SwerveSubsystem drivetrain, Superstructure superstructure){
        return Commands.runOnce(() -> {
            SimulatedArena arena = SimulatedArena.getInstance();

            GamePieceProjectile fuel = new RebuiltFuelOnFly(
                drivetrain.getPose().getTranslation(),
                new Translation2d(
                    Constants.TurretConstants.turretTranslation.getX() * -1,
                    Constants.TurretConstants.turretTranslation.getY()
                ), 
                drivetrain.getSwerveDrive().getRobotVelocity(), 
                drivetrain.getPose().getRotation().rotateBy(superstructure.getAimRotation3d().toRotation2d()), 
                Constants.TurretConstants.turretTranslation.getMeasureZ(),
                // 0.5 times because applying spin to ejected fuel
                superstructure.getTangentialVelocity().times(0.5),
                superstructure.getHoodAngle());
            
            // configure callbacks to visualize the flight trajectory
            fuel.withProjectileTrajectoryDisplayCallBack(
                // callback for when fuel will eventually hit target (if configured)
                (pose3ds) -> Logger.recordOutput("FieldSimulation/Shooter/ProjectileSuccessfulShot",
                    pose3ds.toArray(Pose3d[]::new)),
                // callback for when fuel eventually misses the target, or if no target is configured
                (pose3ds) -> Logger.recordOutput("FuelSimulation/Shooter/ProjectileUnsuccessfulShot",
                    pose3ds.toArray(Pose3d[]::new))
            );
            arena.addGamePieceProjectile(fuel);
        });
    }*/
}
