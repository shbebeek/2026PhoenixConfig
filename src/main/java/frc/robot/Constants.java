// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DriveConstants {
    public static final double DRIVE_SPEED = 1.0;

    // motor ids 1-8
    public static final int[] kDriverIds = new int[8];
    // cancoder ids 9-12
    public static final int[] kEncoderIds = new int[4];
    // pigeon id 0
    public static final int kPigeonId = 0;
  }

  public static class IntakeConstants {
    public static final double INTAKE_SPEED = 1.0;
    
    // motor id 13-14
    public static final Integer kRollerMotorId = 13;
    public static final Boolean kRollerMotorInverted = false;
    public static final Integer kPivotMotorId = 14;
    public static final Boolean kPivotMotorInverted = true;
  }

  public static class HopperConstants {
    public static final double HOPPER_SPEED = 1.0;

    // motor id 15
    public static final Integer kHopperMotorId = 15;
    public static final Boolean kHopperMotorInverted = false; // edit inversions when doing configuration
  }

  public static class FeederConstants {
    public static final double FEEDER_SPEED = 0.75;

    // motor id 16
    public static final Integer kFeederMotorId = 16;
    public static final Boolean kFeederMotorInverted = false;
  }

  public static class TurretConstants {
    public static final double MAX_ONE_DIR_FOV = 90; // degrees
    public static final Translation3d turretTranslation = new Translation3d(-0.205, 0.0, 0.375); // (cranberry)

    // motor id 17
    public static final Integer kTurretMotorId = 17;
    public static final Boolean kTurretMotorInverted = false;

    public static final Angle TURRET_TOLERANCE = Degrees.of(1);
    public static final Double MAX_TURRET_SPEED = 0.1;

    // fixed hood angle
    public static final Integer kHoodAngle = 75;
    public static final Angle HOOD_TOLERANCE = Degrees.of(2);
  }

  public static final class ShooterConstants {
    // motor id 18
    public static final Integer kShooterMotorId = 18;
    public static final Boolean kTurretMotorInverted = false;

    public static final Double kFlywheelDiameter = 4.0;
    public static final AngularVelocity SHOOTER_TOLERANCE = RPM.of(100);

    public static final int SHOOTER_SPEED = 3000; // this is set to 5500 in main
    public static final int SHOOTER_SPEED_FAR = 3250;
    public static final int SHOOTER_SPEED_MIDDLE = 2750;
    public static final int SHOOTER_SPEED_CLOSE = 2000;
    public static final int SHOOTER_SPEED_REALLY_FAR = 4000;
  }

  public static final class ClimberConstants {
    //motor id 19
    public static final Integer kClimberMotorId = 19;
    public static final Boolean kClimberMotorInverted = false;

    public static final double CLIMBER_MOTOR_UP_PERCENT = 0.8;
  }

  public static final class SwerveConstants {
    public static final boolean IS_LIMELIGHT_ENABLED = true;
    // NEED TO CONFIGURE ENCODERS IN SWERVE CONFIG (deploy)
    public static final double MAX_SPEED = Units.feetToMeters(14.5);
  }

  public static enum AimPoints {
    // TODO: get the tower translations and make getTowerPosition function
    RED_HUB(new Translation3d(11.938, 4.034536, 1.5748)),
    RED_OUTPOST(new Translation3d(15.75, 7.25, 0)),
    RED_FAR_SIDE(new Translation3d(15.75, 0.75, 0)),

    BLUE_HUB(new Translation3d(4.5974, 4.034536, 1.5748)),
    BLUE_OUTPOST(new Translation3d(0.75,0.75,0)),
    BLUE_FAR_SIDE(new Translation3d(0.75,7.25,0));

    public final Translation3d value;

    private AimPoints(Translation3d value){
      this.value = value;
    }

    public static final Translation3d getAllianceHubPosition(){
      return DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? RED_HUB.value : BLUE_HUB.value;
    }

    public static final Translation3d getAllianceOutpostPosition(){
      return DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? RED_OUTPOST.value : BLUE_OUTPOST.value;
    }

    public static final Translation3d getAllianceFarSidePosition(){
      return DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? RED_FAR_SIDE.value : BLUE_FAR_SIDE.value;
    }
  }

  public static final class ControllerConstants {
    public static final double DEADBAND = 0.1;

    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final int kPoseControllerPort = 2;
  }

  public static final class VisionConstants {
    // movement speeds for adjusting target pose
    public static final double TRANSLATION_SPEED = 0.05; // meters per periodic cycle
    public static final double ROTATION_SPEED = 2.0; // degrees per periodic cycle
  }
}
