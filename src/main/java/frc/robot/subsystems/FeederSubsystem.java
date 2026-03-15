package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class FeederSubsystem extends SubsystemBase{
    private SparkMax feederController = new SparkMax(Constants.FeederConstants.kFeederMotorId, MotorType.kBrushless);

    private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.OPEN_LOOP)
        .withTelemetry("FeederMotor",TelemetryVerbosity.HIGH)
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(4))) // 4:1 gear reduction
        .withMotorInverted(Constants.HopperConstants.kHopperMotorInverted)
        .withIdleMode(MotorMode.COAST)
        .withStatorCurrentLimit(Amps.of(20));
    
    private SmartMotorController smc = new SparkWrapper(feederController,DCMotor.getNEO(1),smcConfig);

    private final FlyWheelConfig feederConfig = new FlyWheelConfig(smc)
        .withDiameter(Inches.of(4))
        .withMass(Pounds.of(0.5))
        .withUpperSoftLimit(RPM.of(6000))
        .withLowerSoftLimit(RPM.of(-6000))
        .withTelemetry("Feeder",TelemetryVerbosity.HIGH);

    private FlyWheel feeder = new FlyWheel(feederConfig);

    public FeederSubsystem(){

    }

    // run feeder while held, stop when released
    public Command feedCommand(){
        return feeder.set(-Constants.FeederConstants.FEEDER_SPEED).finallyDo(() -> smc.setDutyCycle(0)).withName("Feeder.Feed");
    }

    // stop feeder
    public Command stopCommand(){
        return feeder.set(0).withName("Feeder.Stop");
    }

    @Override
    public void periodic(){
        feeder.updateTelemetry();
    }

    @Override
    public void simulationPeriodic(){
        feeder.simIterate();
    }
}
