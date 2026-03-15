package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.RPM;

import java.util.function.Supplier;

import frc.robot.Constants;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Superstructure extends SubsystemBase{
    public final IntakeSubsystem intake;
    public final HopperSubsystem hopper;
    public final FeederSubsystem feeder;
    public final TurretSubsystem turret;
    public final ShooterSubsystem shooter;
    public final ClimberSubsystem climber;
    public final HoodSubsystem hood;

    // triggers for readiness checks
    private final Trigger isShooterAtSpeed;
    private final Trigger isTurretAtAngle;
    private final Trigger isReadyToShoot;
    private final Trigger isHoodAtAngle;

    private AngularVelocity targetShooterSpeed = RPM.of(0);
    private Angle targetTurretAngle = Degrees.of(0);
    private Angle targetHoodAngle = Degrees.of(0);

    // default aim point is red hub
    private Translation3d aimPoint = Constants.AimPoints.RED_HUB.value;

    public Superstructure(IntakeSubsystem intake, HopperSubsystem hopper, FeederSubsystem feeder,
        TurretSubsystem turret, ShooterSubsystem shooter, ClimberSubsystem climber, HoodSubsystem hood){
        this.intake = intake;
        this.hopper = hopper;
        this.feeder = feeder;
        this.turret = turret;
        this.shooter = shooter;
        this.climber = climber;
        this.hood = hood;

        // create triggers for checking if mechs at targets
        this.isShooterAtSpeed = new Trigger(
            () -> Math.abs(shooter.getSpeed().in(RPM) - targetShooterSpeed.in(RPM))
            < Constants.ShooterConstants.SHOOTER_TOLERANCE.in(RPM));
        
        this.isTurretAtAngle = new Trigger(
            () -> Math.abs(turret.getRawAngle().in(Degrees) - targetTurretAngle.in(Degrees))
            < Constants.TurretConstants.TURRET_TOLERANCE.in(Degrees));
        
        this.isHoodAtAngle = new Trigger(
            () -> Math.abs(hood.getAngle().in(Degrees) - targetHoodAngle.in(Degrees))
            < Constants.TurretConstants.HOOD_TOLERANCE.in(Degrees));
        
        this.isReadyToShoot = isShooterAtSpeed.and(isTurretAtAngle);
    }
    
    // stops all shooting mechanisms from moving
    public Command stopAllShootingCommand(){
        return Commands.parallel(
            shooter.stop().asProxy(),
            turret.set(0).asProxy()).withName("Superstructure.StopAllShooting");
    }

    /**
     * Aims the superstructure to specific targets - used for auto-targeting.
     * @param shooterSpeed Target shooter speed
     * @param turretAngle Target turret angle
     */
    public Command aimCommand(AngularVelocity shooterSpeed, Angle turretAngle, Angle hoodAngle){
        return Commands.runOnce(() -> {
            targetShooterSpeed = shooterSpeed;
            targetTurretAngle = turretAngle;
            targetHoodAngle = hoodAngle;
        }).andThen(
            Commands.parallel(
                shooter.setSpeed(shooterSpeed).asProxy(),
                turret.setAngle(turretAngle).asProxy(),
                hood.setAngle(hoodAngle).asProxy()
            )
        ).withName("Superstructure.Aim");
    }

    public void setShooterSetpoints(AngularVelocity shooterSpeed, Angle turretAngle, Angle hoodAngle){
        targetShooterSpeed = shooterSpeed;
        targetTurretAngle = turretAngle;
        targetHoodAngle = hoodAngle;
    }

    /**
     * Aim superstructure using suppliers - useful for dynamic targeting.
     * @param shooterSpeedSupplier Supplier for target shooter speed
     * @param turretAngleSupplier Supplier for target turret angle
     */
    public Command aimDynamicCommand(
            Supplier<AngularVelocity> shooterSpeedSupplier,
            Supplier<Angle> hoodAngleSupplier,
            Supplier<Angle> turretAngleSupplier){
        return Commands.parallel(
            shooter.setSpeedDynamic(shooterSpeedSupplier).asProxy(),
            turret.setAngleDynamic(turretAngleSupplier).asProxy(),
            hood.setAngleDynamic(hoodAngleSupplier).asProxy()
        ).withName("Superstructure.AimDynamic");
    }

    // waits until superstructure ready to shoot
    public Command waitUntilReadyCommand(){
         return Commands.waitUntil(isReadyToShoot).withName("Superstructure.WaitUntilReady");
    }

    // aims and waits until ready - combines aimDynamic and wait
    public Command aimDynamicAndWaitCommand(AngularVelocity shooterSpeed, Angle hoodAngle, Angle turretAngle){
        return aimDynamicCommand(() -> shooterSpeed, () -> hoodAngle, () -> turretAngle).andThen(waitUntilReadyCommand()).withName("Superstructure.AimDynamicAndWait");
    }

    // auto aim and wait command
    public Command aimAndWaitCommand(AngularVelocity shooterSpeed, Angle hoodAngle, Angle turretAngle){
        return aimCommand(getTargetShooterSpeed(), getTargetTurretAngle(), getTargetHoodAngle()).andThen(waitUntilReadyCommand()).withName("Superstructure.AimAndWait");
    }

    // manual turret control
    public Command setTurretForward(){
        return turret.setAngle(Degrees.of(0)).withName("Superstructure.SetTurretForward");
    }

    public Command setTurretLeft(){
        return turret.setAngle(Degrees.of(45)).withName("Superstructure.SetTurretLeft");
    }

    public Command setTurretRight(){
        return turret.setAngle(Degrees.of(-45)).withName("Superstructure.SetTurretRight");
    }

    public Command rotateTurretLeft(){
        return turret.set(Constants.TurretConstants.MAX_TURRET_SPEED).withName("Superstructure.RotateTurretLeft");
    }

    public Command rotateTurretRight(){
        return turret.set(-Constants.TurretConstants.MAX_TURRET_SPEED).withName("Superstructure.RotateTurretRight");
    }

    public Command stopTurret(){
        return turret.set(0).withName("Superstructure.StopTurret");
    }

    // getters for current state
    public AngularVelocity getShooterSpeed(){
        return shooter.getSpeed();
    }

    public Angle getTurretAngle(){
        return turret.getRawAngle();
    }

    public Angle getHoodAngle(){
        return hood.getAngle();
    }

    public AngularVelocity getTargetShooterSpeed(){
        return targetShooterSpeed;
    }

    public Angle getTargetTurretAngle(){
        return targetTurretAngle;
    }

    public Angle getTargetHoodAngle(){
        return targetHoodAngle;
    }

    public Translation3d getAimPoint(){
        return aimPoint;
    }

    public void setAimPoint(Translation3d newAimPoint){
        this.aimPoint = newAimPoint;
    }

    public Rotation3d getAimRotation3d(){
        return new Rotation3d(
            Degrees.of(0),
            hood.getAngle().unaryMinus(),
            turret.getRobotAdjustedAngle()
        );
    }

    // run intake roller (need stopIntake)
    public Command intakeCommand(){
        return intake.intakeCommand().withName("Superstructure.Intake");
    }

    public Command deployIntakeCommand(){
        return intake.deployIntake().withName("Superstructure.DeployIntake");
    }

    public Command returnIntakeCommand(){
        return intake.returnIntake().withName("Superstructure.ReturnIntake");
    }

    public Command stopIntakePivot(){
        return intake.stopPivot().withName("Superstructure.StopIntakePivot");
    }

    // stop intake
    public Command stopIntakeCommand(){
        return intake.stopIntake().withName("Superstructure.StopIntake");
    }

    // eject while held
    public Command ejectCommand(){
        return intake.ejectCommand().withName("Supersctructure.IntakeEject");
    }

    // run hopper forward while held
    public Command hopperFeedCommand(){
        return hopper.feedCommand().withName("Superstructure.Feed");
    }

    // outtkae hopper while held
    public Command hopperEjectCommand(){
        return hopper.backFeedCommand().withName("Superstructure.HopperEject");
    }

    // run feeder while held
    public Command feederFeedCommand(){
        return feeder.feedCommand().withName("Superstructure.FeederFeed");
    }

    public Command feederStopCommand(){
        return feeder.stopCommand().withName("Superstructure.FeederStop");
    }

    public Command feedAllCommand(){
        return Commands.parallel(
            hopper.feedCommand().asProxy(),
            feeder.feedCommand().asProxy()
        ).withName("Superstructure.FullFeed");
    }

    // full eject while held
    public Command ejectAllCommand(){
        return Commands.parallel(
            hopper.backFeedCommand().asProxy(),
            intake.backFeedAndRollCommand().asProxy()
        ).withName("Superstructure.FullEject");
    }

    public Command intakeBounceCommand(){
        return Commands.repeatingSequence(
            Commands.runOnce(() -> intake.setPivotAngle(Degrees.of(115)).asProxy().withName("Superstructure.IntakeBounce.Deploy")),
            Commands.waitSeconds(0.5),
            Commands.runOnce(() -> intake.setPivotAngle(Degrees.of(59)).asProxy().withName("Superstructure.IntakeBounce.Feed")),
            Commands.waitSeconds(0.5)
        ).withName("Superstructure.IntakeBounce");
    }

    public Command stopFeedingAllCommand(){
        return Commands.parallel(
            //intake.deployAndRollCommand().asProxy(),
            hopper.stopCommand().asProxy(),
            feeder.stopCommand().asProxy()
        ).withName("Superstructure.FullStopFeeding");
    }

    // set intake pivot angle
    public Command setIntakePivotAngle(Angle angle){
        return intake.setPivotAngle(angle).withName("Superstructure.SetIntakePivotAngle");
    }

    // deploy and roll while held
    public Command setIntakeDeployAndRoll(){
        return intake.deployAndRollCommand().withName("Superstructure.SetIntakeDeployAndRoll");
    }

    // stow intake
    public Command setIntakeStow(){
        return intake.stowIntake().withName("Superstructure.SetIntakeStow");
    }

    // shoot - spins up shooter
    public Command shootCommand(){
        return shooter.spinUp().withName("Superstructure.Shoot");
    }

    // shoot slow
    public Command shootSlowCommand(){
        return shooter.spinUpSlow().withName("Superstructure.ShootSlow");
    }

    // re-zero both intake pivot and turret
    public Command rezeroIntakePivotAndTurretCommand(){
        return Commands.parallel(
            turret.rezero().withName("Superstructure.rezeroTurret"),
            intake.rezero().withName("Superstructure.rezeroIntakePivot")
        ).withName("Superstructure.RezeroIntakePivotAndTurret");
    }

    public Command moveClimberUp(){
        return climber.climbUp().withName("Superstructure.ClimbUp");
    }

    public Command moveClimberDown(){
        return climber.climbDown().withName("Superstructure.ClimbDown");
    }

    @Override
    public void periodic(){
        // superstructure doesn't need periodic updates since subsystems update on their own
        String shooterOut = "S:" + isShooterAtSpeed.getAsBoolean() + "(" + Math.round(shooter.getSpeed().in(RPM))
            + "/" + Math.round(targetShooterSpeed.in(RPM)) + ")";

        String turretOut = "T:" + isTurretAtAngle.getAsBoolean() + "(" + Math.round(turret.getRawAngle().in(Degrees))
            + "/" + Math.round(targetTurretAngle.in(Degrees)) + ")";

        String hoodOut = "H:" + isHoodAtAngle.getAsBoolean() + "(" + Math.round(hood.getAngle().in(Degrees))
            + "/" + Math.round(targetHoodAngle.in(Degrees)) + ")";

        String readyOut = "R:" + isReadyToShoot.getAsBoolean();

        System.out.println(shooterOut + " " + turretOut + " " + hoodOut + " " + readyOut);
    }

    // not entirely sure of the use of this function but i'm leaving it in here

    public Command useRequirement(){
        return runOnce(() -> {
        });
    }

    public Pose3d getShooterPose(){
        // position of shooter relative to front of robot
        // rotation element based on turret and hood angle
        // (hood angle need to find from cad) (these taken from CA)

        return new Pose3d(new Translation3d(
            Meter.of(-0.3),
            Meter.of(0),
            Meter.of(0.6)),
            getAimRotation3d()
        );
    }

    public LinearVelocity getTangentialVelocity(){
        return shooter.getTangentialVelocity();
    }
}
