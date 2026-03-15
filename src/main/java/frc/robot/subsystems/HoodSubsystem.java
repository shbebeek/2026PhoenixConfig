package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.Supplier;

import frc.robot.Constants;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HoodSubsystem extends SubsystemBase{
    // if we add hood control then this is the subsystem for it
    // otherwise i'm adding this in for the poses and translations
    //  needed for integration in superstructure

    public HoodSubsystem(){

    }

    public Command setAngle(Angle angle){
        // return hood.setAngle(angle);
        return Commands.runOnce(() -> {});
    }

    public Command setAngleDynamic(Supplier<Angle> hoodAngleSupplier){
        // hood.setAngle(hoodAngleSupplier)
        return Commands.run(() -> {});
    }

    public Command stow(){
        return setAngle(Degrees.of(0));
    }

    public Command max(){
        return setAngle(Degrees.of(90));
    }

    public Angle getAngle(){
        return Degrees.of(Constants.TurretConstants.kHoodAngle); // fixed hood angle
    }

    public Command set(double dutyCycle){
        return Commands.runOnce(() -> {});
    }

    public Command sysId(){
        // return hood.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(10));
        return Commands.runOnce(() -> {});
    }

    @Override
    public void periodic(){

    }

    @Override
    public void simulationPeriodic(){

    }
}
