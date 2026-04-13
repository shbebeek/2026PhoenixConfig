package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

//import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
import yams.motorcontrollers.remote.TalonFXWrapper;

public class ShooterSubsystem extends SubsystemBase{
    // 1 Kraken x60, 4in shooter wheels

    private final TalonFX motorController = new TalonFX(Constants.ShooterConstants.kShooterMotorId);
    
    private final SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(0.00936,0,0)
        .withFeedforward(new SimpleMotorFeedforward(0.191, 0.11858, 0.0))
        .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
        .withMotorInverted(Constants.ShooterConstants.kTurretMotorInverted)
        .withIdleMode(MotorMode.COAST)
        .withStatorCurrentLimit(Amps.of(40));

    private final SmartMotorController smc = new TalonFXWrapper(motorController, DCMotor.getKrakenX60(1), smcConfig);

    private final FlyWheelConfig shooterConfig = new FlyWheelConfig(smc)
        .withDiameter(Inches.of(Constants.ShooterConstants.kFlywheelDiameter))
        .withMass(Pounds.of(1))
        .withUpperSoftLimit(RPM.of(6000))
        .withLowerSoftLimit(RPM.of(0))
        .withTelemetry("Shooter",TelemetryVerbosity.HIGH);
    
    private final FlyWheel shooter = new FlyWheel(shooterConfig);

    private int currentSetpoint = -Constants.ShooterConstants.SHOOTER_SPEED;

    public ShooterSubsystem(){

    }

    public Command setSpeed(AngularVelocity speed){
        return shooter.setSpeed(speed);
    }

    public Command setSpeedDynamic(Supplier<AngularVelocity> speedSupplier){
        return shooter.setSpeed(speedSupplier);
    }

    public Command spinUp(int speed){
        return setSpeed(RPM.of(speed));
    }

    public Command spinUpSlow(){
        return setSpeed(RPM.of(-500));
    }

    public Command stop(){
        return setSpeed(RPM.of(0));
    }

    public Command mediumBumpUp(){
        currentSetpoint -= 50;
        return setSpeed(RPM.of(currentSetpoint));
    }

    public Command mediumBumpDown(){
        currentSetpoint += 50;
        return setSpeed(RPM.of(currentSetpoint));
    }

    public Command lowBumpUp(){
        currentSetpoint -= 25;
        return setSpeed(RPM.of(currentSetpoint));
    }

    public Command lowBumpDown(){
        currentSetpoint += 25;
        return setSpeed(RPM.of(currentSetpoint));
    }

    public AngularVelocity getSpeed(){
        return shooter.getSpeed();
    }

    public Command sysId(){
        return shooter.sysId(Volts.of(12), Volts.of(3).per(Second), Seconds.of(7));
    }

    @Override
    public void periodic(){
        //Logger.recordOutput("Shooter/Velocity", motorController.getVelocity().getValue());
        SmartDashboard.putNumber("Current Modified Setpoint",currentSetpoint);
        SmartDashboard.putNumber("Tower Shooting Setpoint",-Constants.ShooterConstants.SHOOTER_SPEED);
    }

    @Override
    public void simulationPeriodic(){
        shooter.simIterate();
    }

    private Distance wheelRadius(){
        return Inches.of(Constants.ShooterConstants.kFlywheelDiameter).div(2);
    }

    public LinearVelocity getTangentialVelocity(){
        // tangential velocity at edge of wheel converted to linear velocity (v = wr)
        return MetersPerSecond.of(getSpeed().in(RadiansPerSecond) * wheelRadius().in(Meters));
    }
}
  