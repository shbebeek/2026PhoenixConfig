package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;

import frc.robot.subsystems.Superstructure;

public class OperatorControls {

    public static void configure(int port, Superstructure superstructure){
        if(Robot.isReal()){
            CommandXboxController controller = new CommandXboxController(port);

            controller.povDown().whileTrue(superstructure.deployIntakeCommand().finallyDo(() -> superstructure.stopIntakePivot().schedule()).withName("DriverControls.Deploy"));
            controller.povUp().whileTrue(superstructure.returnIntakeCommand().finallyDo(() -> superstructure.stopIntakePivot().schedule()));
            controller.start().whileTrue(superstructure.shootReallyFarCommand());
            controller.povRight().whileTrue(superstructure.stopAllShootingCommand());
            controller.y().whileTrue(superstructure.shootMiddleCommand());
            controller.x().whileTrue(superstructure.shootCommand());
            controller.b().whileTrue(superstructure.shootCloseCommand());
            controller.a().whileTrue(superstructure.shootFarCommand());
            controller.rightBumper().whileTrue(superstructure.rotateTurretLeft().finallyDo(() -> superstructure.stopTurret().schedule()));
            controller.leftBumper().whileTrue(superstructure.rotateTurretRight().finallyDo(() -> superstructure.stopTurret().schedule()));
            // TODO: code in the vision for auto-targeting to tower (button x)
            controller.rightTrigger().whileTrue(superstructure.ejectCommand().finallyDo(() -> superstructure.stopIntakeCommand().schedule()));        }
    }
}
