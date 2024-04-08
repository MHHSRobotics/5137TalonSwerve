package frc.robot.Commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.Swerve_Constants;
import frc.robot.Subsystems.Swerve;
import swervelib.SwerveDrive;

public class Swerve_Commands {
   
    private final Swerve swerve;

    public Swerve_Commands(Swerve swerveSubsystem){
        this.swerve = swerveSubsystem;
    }

    public InstantCommand drive(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation, BooleanSupplier fieldRelative) {
        return new InstantCommand(
            () -> swerve.drive(
                new Translation2d(
                Math.pow(translationX.getAsDouble(), 3)*Swerve_Constants.maxSpeed,
                Math.pow(translationY.getAsDouble(), 3)*Swerve_Constants.maxSpeed), 
                Math.pow(rotation.getAsDouble(), 3)*Swerve_Constants.maxAngularSpeed, 
                fieldRelative.getAsBoolean()),
            swerve);
    }

    public InstantCommand zeroGyro() {
        return new InstantCommand(() -> swerve.zeroGyro(), swerve);
    }

    public Command runAuto() {
        return swerve.getAuto();
    }
}