package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class TurretVisionSubsystem extends SubsystemBase {
    private final SparkMax turretMotor = new SparkMax(Constants.TurretConstants.kTurretMotorIdFake, MotorType.kBrushless);

    private final PIDController turretPID = new PIDController(0.005, 0.0, 0.0001);

    private double turretAngleDeg = turretMotor.getEncoder().getPosition();

    private final CommandSwerveDrivetrain drivetrain;

    private static final Translation3d RED_HUB = new Translation3d(11.938, 4.034536, 1.5748);
    private static final Translation3d BLUE_HUB = new Translation3d(4.5974, 4.034536, 1.5748);

    public TurretVisionSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        turretPID.enableContinuousInput(-180, 180);
        turretPID.setTolerance(1.0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Turret Angle", getTurretAngle());
    }

    public void autoAim() {
        if (Limelight.hasTarget()) {
            Pose2d visionPose = Limelight.getPose2d();
            double timestamp = Timer.getFPGATimestamp() - Limelight.getLatencySeconds();

            drivetrain.addVisionMeasurement(visionPose, timestamp);
        }

        Pose2d robotPose = drivetrain.getState().Pose;

        double dx = RED_HUB.getX() - robotPose.getX();
        double dy = RED_HUB.getY() - robotPose.getY();

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
        turretMotor.set(output);
        turretAngleDeg = turretMotor.getEncoder().getPosition();
    }

    public void stop() {
        turretMotor.set(0);
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