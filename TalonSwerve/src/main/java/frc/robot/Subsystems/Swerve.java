package frc.robot.Subsystems;

import java.io.File;
import java.io.IOException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Swerve_Constants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class Swerve extends SubsystemBase{

    private final SwerveDrive swerveDrive;
    private SendableChooser<Command> autoChooser;

    public Swerve(File directory){
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        try{
            swerveDrive = new SwerveParser(directory).createSwerveDrive(Swerve_Constants.maxSpeed);
        } catch (Exception e){
            throw new RuntimeException(e);
        }
        swerveDrive.setHeadingCorrection(false); 
        swerveDrive.setCosineCompensator(!SwerveDriveTelemetry.isSimulation); 
        setUpPathPlanner();
    }

    public void drive(Translation2d translation2d, double rotationSpeed, boolean fieldRelative) {
        swerveDrive.drive(translation2d, rotationSpeed, fieldRelative, true);
    }

    public void setChassisSpeeds(ChassisSpeeds velocity) {
        swerveDrive.setChassisSpeeds(velocity);
    }

    public void lock()
    {
        swerveDrive.lockPose();
    }

    public Command getAuto() {
        return autoChooser.getSelected();
    }

    public void resetOdometry(Pose2d pose) {
        swerveDrive.resetOdometry(pose);
    }

    public void zeroGyro() {
        swerveDrive.zeroGyro();  
        if(isRedAlliance()){
            resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
        }      
    }

    public ChassisSpeeds getRobotVelocity() {
        return swerveDrive.getRobotVelocity();
    }

    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    public void addVisionMeasurement(Pose2d pose, double timestamp){ 
        swerveDrive.addVisionMeasurement(pose, timestamp);
    }

    public void setUpPathPlanner() {
        AutoBuilder.configureHolonomic(
            this::getPose,
            this::resetOdometry,
            this::getRobotVelocity,
            this::setChassisSpeeds,
            new HolonomicPathFollowerConfig(
                new PIDConstants(1, 0.0, 0.4),
                new PIDConstants(2, 0.0, 0.4),
                Swerve_Constants.maxModuleSpeed,
                swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters(),
                new ReplanningConfig()),
            () -> {
                var alliance = DriverStation.getAlliance();
                if(alliance.isPresent())
                {
                return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this);
        autoChooser = AutoBuilder.buildAutoChooser("Mid2");
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public boolean isRedAlliance(){
        return DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() == DriverStation.Alliance.Red : false; 
    }

}
