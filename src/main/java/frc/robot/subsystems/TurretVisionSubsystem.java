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
import frc.robot.subsystems.CommandSwerveDrivetrain; // your phoenix drivetrain

public class TurretVisionSubsystem extends SubsystemBase {

    // =============================
    // HARDWARE
    // =============================
    private final SparkMax turretMotor = new SparkMax(Constants.TurretConstants.kTurretMotorId, MotorType.kBrushless);

    // =============================
    // CONTROL
    // =============================
    private final PIDController turretPID = new PIDController(0.02, 0.0, 0.001);

    // Simulated encoder position (replace with real encoder if you have one)
    private double turretAngleDeg = turretMotor.getEncoder().getPosition();

    // =============================
    // DEPENDENCIES
    // =============================
    private final CommandSwerveDrivetrain drivetrain;

    // =============================
    // FIELD CONSTANTS (EDIT THIS)
    // =============================
    private static final Translation2d HUB_POSITION = new Translation2d(8.25, 4.1);

    public TurretVisionSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        turretPID.enableContinuousInput(-180, 180);
        turretPID.setTolerance(1.0);
    }

    // =============================
    // PERIODIC
    // =============================
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Turret Angle", getTurretAngle());
    }

    // =============================
    // AUTO AIM LOGIC
    // =============================
    public void autoAim() {

        // 1. Inject vision into swerve pose estimator
        if (Limelight.hasTarget()) {
            Pose2d visionPose = Limelight.getPose2d();
            double timestamp = Timer.getFPGATimestamp() - Limelight.getLatencySeconds();

            drivetrain.addVisionMeasurement(visionPose, timestamp);
        }

        // 2. Get fused pose (swerve + vision)
        Pose2d robotPose = drivetrain.getState().Pose;

        // 3. Compute angle to hub
        double dx = HUB_POSITION.getX() - robotPose.getX();
        double dy = HUB_POSITION.getY() - robotPose.getY();

        double targetAngle = Math.toDegrees(Math.atan2(dy, dx));

        // 4. Convert to turret-relative
        double robotHeading = robotPose.getRotation().getDegrees();
        double turretSetpoint = targetAngle - robotHeading;

        turretSetpoint = Math.toDegrees(
            MathUtil.angleModulus(Math.toRadians(turretSetpoint))
        );

        // 5. PID control
        double output = turretPID.calculate(getTurretAngle(), turretSetpoint);

        setTurretMotor(output);

        // =============================
        // DEBUG
        // =============================
        SmartDashboard.putNumber("Robot X", robotPose.getX());
        SmartDashboard.putNumber("Robot Y", robotPose.getY());
        SmartDashboard.putNumber("Target Angle", targetAngle);
        SmartDashboard.putNumber("Turret Setpoint", turretSetpoint);
    }

    // =============================
    // MOTOR CONTROL
    // =============================
    private void setTurretMotor(double output) {
        output = MathUtil.clamp(output, -0.5, 0.5);

        turretMotor.set(output);

        // Replace with real encoder feedback
        turretAngleDeg = turretMotor.getEncoder().getPosition();
    }

    public void stop() {
        turretMotor.set(0);
    }

    public double getTurretAngle() {
        return turretAngleDeg;
    }

    // =============================
    // AUTO AIM COMMAND
    // =============================
    public Command autoAimCommand() {
        return new RunCommand(() -> autoAim(), this)
            .finallyDo(() -> stop());
    }

    // =============================
    // LIMELIGHT HELPER (MegaTag2)
    // =============================
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