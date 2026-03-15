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
            // TODO: code in the vision for auto-targeting to tower (button x)
        }
    }
}
