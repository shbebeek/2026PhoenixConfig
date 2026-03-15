package frc.robot.controls;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;

// controls for moving a visual target pose on the field
// target pose can be used for autonomous navigation (drive-to-pose)
public class PoseControls {
    /*
    private static Pose2d targetPose = new Pose2d(new Translation2d(4,4), Rotation2d.fromDegrees(0));
    
    /**
     * Configure pose controls on given controller
     * @param controller The controller to bind pose adjustment controls to
     * @param drivetrain The swerve subsystem for updating the field visualization
     */
    /*
    public static void configure(int port){
        CommandXboxController controller = new CommandXboxController(port);

        // update field visualization with initial pose
        updateFieldPose(drivetrain);

        /**
         * continuous joystick control for target pose adjustment
         * left stick x/y: translation, right stick x: rotation
         * using ignoringDisable(true) so it runs even when disabled, and no reqs so it doesn't conflict
         */
        /*Commands.run(() -> {
            double leftX = controller.getLeftX();
            double leftY = controller.getLeftY();
            double rightX = controller.getRightX();

            // apply deadband
            if(Math.abs(leftX) < Constants.ControllerConstants.DEADBAND) leftX = 0;
            if(Math.abs(leftY) < Constants.ControllerConstants.DEADBAND) leftY = 0;
            if(Math.abs(rightX) < Constants.ControllerConstants.DEADBAND) rightX = 0;
            
            // alliance-relative controls:
            // forward (negative leftY) should move away from driver station
            // blue alliance: driver station at x = 0, so forward = +X
            // red alliance: driver station at x = 16.5m, so forward = -X (flip X direction)
            // left (negative leftX) should move toward left side of field from driver POV
            // blue alliance: left = +Y
            // red alliance: left = -Y (flip Y direction)
            boolean isRedAlliance = DriverStation.getAlliance()
                .map(alliance -> alliance == DriverStation.Alliance.Red)
                .orElse(false);

            double allianceMultiplier = isRedAlliance ? -1.0 : 1.0;

            // adjust target pose based on joystick input (alliance-relative)
            if(leftX != 0 || leftY != 0){
                // -leftY = forward motion, leftX = strafe right
                // apply alliance multiplier to make controls relative to driver station
                adjustTargetTranslation(
                    -leftY * Constants.VisionConstants.TRANSLATION_SPEED * allianceMultiplier, // forward/back -> x
                    -leftX * Constants.VisionConstants.TRANSLATION_SPEED * allianceMultiplier); // left/right -> y
            }
            if(rightX != 0){
                adjustTargetRotation(-rightX * Constants.VisionConstants.ROTATION_SPEED);
            }

            updateFieldPose(drivetrain);
        }).ignoringDisable(true).withName("PoseControls.Update").schedule();

        // Y button: reset target pose to robot's current position
        controller.back().onTrue(Commands.runOnce(() -> {
            targetPose = drivetrain.getPose();
            updateFieldPose(drivetrain);
        }).ignoringDisable(true));
    }

    /**
     * update field visualization with current target pose
     * @param drivetrain The swerve subsystem containing the field
     
    private static void updateFieldPose(SwerveSubsystem drivetrain){
        drivetrain.getSwerveDrive().field.getObject("targetPose").setPose(targetPose);
    }

    /**
     * Adjust target translation by given delta values
     * @param deltaX in meters
     * @param deltaY in meters
     
    private static void adjustTargetTranslation(double deltaX, double deltaY){
        targetPose = new Pose2d(
            targetPose.getX() + deltaX,
            targetPose.getY() + deltaY,
            targetPose.getRotation());
    }

    /**
     * Adjust target rotation by given delta
     * @param deltaDegrees
     
    private static void adjustTargetRotation(double deltaDegrees){
        targetPose = new Pose2d(
            targetPose.getTranslation(),
            targetPose.getRotation().plus(Rotation2d.fromDegrees(deltaDegrees)));
    }

    /**
     * get current target pose
     * @return current target pose
     
    public static Pose2d getTargetPose(){
        return targetPose;
    }

    /**
     * get supplier for current target pose
     * useful for commands that need dynamic pose reference
     * @return supplier that returns current target pose
     
    public static Supplier<Pose2d> getTargetPoseSupplier(){
        return () -> targetPose;
    }

    /**
     * set target pose directly
     * @param pose new target pose
     
    public static void setTargetPose(Pose2d pose){
        targetPose = pose;
    }

    /**
     * target pose and update field visualization
     * @param pose new target pose
     * @param drivetrain swerve subsystem for updating field
     
    public static void setTargetPose(Pose2d pose, SwerveSubsystem drivetrain){
        targetPose = pose;
        updateFieldPose(drivetrain);
    }*/
}
