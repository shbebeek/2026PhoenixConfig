package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

//import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.config.MechanismPositionConfig.Plane;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class TurretSubsystem extends SubsystemBase{
    private SparkMax turretController = new SparkMax(Constants.TurretConstants.kTurretMotorId, MotorType.kBrushless);

    private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(15.0,0,0, DegreesPerSecond.of(2440),DegreesPerSecondPerSecond.of(2440))
        .withFeedforward(new SimpleMotorFeedforward(0,7.5,0))
        .withTelemetry("TurretMotor",TelemetryVerbosity.HIGH)
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(4,10)))
        .withMotorInverted(Constants.TurretConstants.kTurretMotorInverted)
        .withIdleMode(MotorMode.COAST)
        .withSoftLimit(Degrees.of(-Constants.TurretConstants.MAX_ONE_DIR_FOV), Degrees.of(Constants.TurretConstants.MAX_ONE_DIR_FOV))
        .withStatorCurrentLimit(Amps.of(10))
        .withClosedLoopRampRate(Seconds.of(0.1))
        .withOpenLoopRampRate(Seconds.of(0.1));

    private SmartMotorController smc = new SparkWrapper(turretController,DCMotor.getNeo550(1),smcConfig);

    private final PivotConfig turretConfig = new PivotConfig(smc)
        .withHardLimit(Degrees.of(-Constants.TurretConstants.MAX_ONE_DIR_FOV - 5), Degrees.of(Constants.TurretConstants.MAX_ONE_DIR_FOV + 5))
        .withStartingPosition(Degrees.of(0))
        .withMOI(0.05)
        .withTelemetry("Turret", TelemetryVerbosity.HIGH)
        .withMechanismPositionConfig(
            new MechanismPositionConfig().withMovementPlane(Plane.XY).withRelativePosition(Constants.TurretConstants.turretTranslation));
    
    private Pivot turret = new Pivot(turretConfig);

    // start of vision
    private final PIDController turretPID = new PIDController(0.005, 0.0, 0.0001);

    private double turretAngleDeg = turretController.getEncoder().getPosition();

    private final CommandSwerveDrivetrain drivetrain;

    private static final Translation3d RED_HUB = new Translation3d(11.938, 4.034536, 1.5748);
    private static final Translation3d BLUE_HUB = new Translation3d(4.5974, 4.034536, 1.5748);


    public TurretSubsystem (CommandSwerveDrivetrain drivetrain){
        this.drivetrain = drivetrain;
        turretPID.enableContinuousInput(-180, 180);
        turretPID.setTolerance(1.0);
    }

    // set angle via degree input
    public Command setAngle(Angle angle){
        return turret.setAngle(angle);
    }

    // set angle via suppliers from telemetry (NT)
    public Command setAngleDynamic(Supplier<Angle> turretAngleSupplier){
        return turret.setAngle(turretAngleSupplier);
    }

    public Command center(){
        return turret.setAngle(Degrees.of(0));
    }

    public Angle getRobotAdjustedAngle(){
        // returns turret angle in robot's coordinate frame
        // if turret mounted backward, add 180 degrees, else ignore this
        return turret.getAngle().plus(Degrees.of(180));
    }

    public Angle getRawAngle(){
        return turret.getAngle();
    }

    public Command set(double dutyCycle){
        return turret.set(dutyCycle);
    }

    public Command rezero(){
        return Commands.runOnce(() -> turretController.getEncoder().setPosition(0), this).withName("Turret.Rezero");
    }

    public Command sysId(){
        return turret.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(10));
    }

    @Override
    public void periodic(){
        turret.updateTelemetry();
        SmartDashboard.putNumber("Turret Angle", getTurretAngle());

        /*Logger.recordOutput("ASCalibration/FinalComponentPoses", new Pose3d[] {
            new Pose3d(
                Constants.TurretConstants.turretTranslation,
                new Rotation3d(0,0,turret.getAngle().in(Radians))
            )
        });*/
    }

    @Override
    public void simulationPeriodic(){
        turret.simIterate();
    }





    // start of vision
    public void autoAim() {
        if (Limelight.hasTarget()) {
            Pose2d visionPose = Limelight.getPose2d();
            double timestamp = Timer.getFPGATimestamp() - Limelight.getLatencySeconds();

            drivetrain.addVisionMeasurement(visionPose, timestamp);
        }

        Pose2d robotPose = drivetrain.getState().Pose;

        double dx = BLUE_HUB.getX() - robotPose.getX();
        double dy = BLUE_HUB.getY() - robotPose.getY();

        double targetAngle = Math.toDegrees(Math.atan2(dy, dx));

        double robotHeading = robotPose.getRotation().getDegrees();
        double turretSetpoint = targetAngle - robotHeading;

        turretSetpoint = Math.toDegrees(
            MathUtil.angleModulus(Math.toRadians(turretSetpoint))
        );

        double output = turretPID.calculate(getTurretAngle(), turretSetpoint);

        setTurretMotor(output);

        SmartDashboard.putNumber("Robot X", robotPose.getX());
        SmartDashboard.putNumber("Robot Y", robotPose.getY());
        SmartDashboard.putNumber("Target Angle", targetAngle);
        SmartDashboard.putNumber("Turret Setpoint", turretSetpoint);
    }

    private void setTurretMotor(double output) {
        output = MathUtil.clamp(output, -0.5, 0.5);
        turretController.set(output);
        turretAngleDeg = turretController.getEncoder().getPosition();
    }

    public void stop() {
        turretController.set(0);
    }

    public double getTurretAngle() {
        return turretAngleDeg;
    }

    public Command autoAimCommand() {
        return new RunCommand(() -> autoAim(), this)
            .finallyDo(() -> stop());
    }

    public static class Limelight {
        private static final NetworkTable table =
            NetworkTableInstance.getDefault().getTable("limelight");

        public static boolean hasTarget() {
            return table.getEntry("tv").getDouble(0) == 1;
        }

        public static Pose2d getPose2d() {
            double[] p = table.getEntry("botpose_wpiblue")
                .getDoubleArray(new double[6]);

            return new Pose2d(
                p[0],
                p[1],
                Rotation2d.fromDegrees(p[5])
            );
        }

        public static double getLatencySeconds() {
            double tl = table.getEntry("tl").getDouble(0);
            return tl / 1000.0;
        }
    }
}
