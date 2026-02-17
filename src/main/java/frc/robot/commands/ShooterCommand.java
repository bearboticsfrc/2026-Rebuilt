package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.ShootHood;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.subsystems.TrajectoryCalculator.ShotCalculator;

public class ShooterCommand {

    private final ShootHood shootHood = new ShootHood();
    private final Flywheel flywheel = new Flywheel();
    private final Turret turret = new Turret();
    // private final CommandSwerveDrivetrain drivetrain = new CommandSwerveDrivetrain();
    private final ShotCalculator shotCalculator = new ShotCalculator(drivetrain.getPose(), drivetrain.getChassisSpeeds());

    public ShooterCommand() {}


    public Command shootAt() {
        return new InstantCommand(() -> {
            double[] shotCalculations = shotCalculator.ShootOnMoveSolver(shotCalculator.targetLocation());
            double flywheelRPM = shotCalculator.flywheelRPMFromVelocity(shotCalculations[1]);
            shootHood.setHoodAngle(Degrees.of(shotCalculations[2]));
            flywheel.runFlywheel(flywheelRPM);
            // turret.setAngle(Degrees.of(shotCalculations[3]));
        }, shootHood, flywheel/*, turret*/);
    }
}
