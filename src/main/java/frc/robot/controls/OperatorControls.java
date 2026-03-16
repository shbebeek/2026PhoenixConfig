package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;

import frc.robot.subsystems.Superstructure;

public class OperatorControls {

    public static void configure(int port, Superstructure superstructure){
        if(Robot.isReal()){
            CommandXboxController controller = new CommandXboxController(port);

            controller.y().whileTrue(superstructure.moveClimberDown());
            controller.a().whileTrue(superstructure.moveClimberUp());
            controller.povDown().whileTrue(superstructure.deployIntakeCommand().finallyDo(() -> superstructure.stopIntakePivot().schedule()).withName("DriverControls.Deploy"));
            controller.povUp().whileTrue(superstructure.returnIntakeCommand().finallyDo(() -> superstructure.stopIntakePivot().schedule()));
            controller.start().whileTrue(superstructure.shootReallyFarCommand());
            controller.povRight().whileTrue(superstructure.stopAllShootingCommand());
            // TODO: code in the vision for auto-targeting to tower (button x)
        }
    }
}
