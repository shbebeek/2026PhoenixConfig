package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase{
    public SparkMax climberController;

    public ClimberSubsystem(){
        climberController = new SparkMax(Constants.ClimberConstants.kClimberMotorId, MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(40);
        config.idleMode(IdleMode.kBrake);
        climberController.configure(config,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    }

    public void setClimber(double power){
        climberController.set(power);
    }

    public void stop(){
        climberController.set(0);
    }

    public Command climbUp(){
        return Commands.run(() -> setClimber(Constants.ClimberConstants.CLIMBER_MOTOR_UP_PERCENT), this).finallyDo(() -> stop()).withName("Climber.ClimbUp");
    }

    public Command climbDown(){
        return Commands.run(() -> setClimber(-Constants.ClimberConstants.CLIMBER_MOTOR_UP_PERCENT), this).finallyDo(() -> stop()).withName("Climber.ClimbDown");
    }

    public Command stopMotion(){
        return Commands.run(() -> stop(), this).withName("Climber.StopMotion");
    }

    @Override
    public void periodic(){
  
    }

    @Override
    public void simulationPeriodic(){

    }
}
