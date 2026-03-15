// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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

public class HopperSubsystem extends SubsystemBase {
  private SparkMax hopperController = new SparkMax(Constants.HopperConstants.kHopperMotorId, MotorType.kBrushed);

  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
    .withControlMode(ControlMode.OPEN_LOOP)
    .withTelemetry("HopperMotor", TelemetryVerbosity.HIGH)
    .withGearing(new MechanismGearing(GearBox.fromReductionStages(4))) // 4:1 gear reduction
    .withMotorInverted(Constants.HopperConstants.kHopperMotorInverted)
    .withIdleMode(MotorMode.COAST)
    .withStatorCurrentLimit(Amps.of(40));
  
  private SmartMotorController smc = new SparkWrapper(hopperController, DCMotor.getCIM(1), smcConfig);

  private final FlyWheelConfig hopperConfig = new FlyWheelConfig(smc)
    .withDiameter(Inches.of(4))
    .withMass(Pounds.of(0.5))
    .withUpperSoftLimit(RPM.of(6000))
    .withLowerSoftLimit(RPM.of(-6000))
    .withTelemetry("Hopper", TelemetryVerbosity.HIGH);

  private FlyWheel hopper = new FlyWheel(hopperConfig);
  
  public HopperSubsystem() {
    
  }

  // run hopper forward while held
  public Command feedCommand(){
    return hopper.set(Constants.HopperConstants.HOPPER_SPEED).finallyDo(() -> smc.setDutyCycle(0)).withName("Hopper.Feed");
  }

  // run hopper in reverse while held
  public Command backFeedCommand(){
    return hopper.set(-Constants.HopperConstants.HOPPER_SPEED).finallyDo(() -> smc.setDutyCycle(0)).withName("Hopper.Reverse");
  }

  // stop hopper
  public Command stopCommand(){
    return hopper.set(0).withName("Hopper.Stop");
  }

  @Override
  public void periodic() {
    hopper.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    hopper.simIterate();
  }
}
