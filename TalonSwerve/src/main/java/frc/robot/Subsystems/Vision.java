package frc.robot.Subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Vision_Constants;

public class Vision extends SubsystemBase{
    
    private final PhotonCamera objectCamera;
    private double latestMetersToNote;
    private Translation2d latestTranslationToNote;

    public Vision(){
        objectCamera = new PhotonCamera("objectCamera");
        latestMetersToNote = 0;
        latestTranslationToNote = new Translation2d();
    }

    public double getMetersToNote()
    {
      Transform3d robotToCamera = Vision_Constants.objectCameraToRobot;
      var result = objectCamera.getLatestResult();
      if(result.hasTargets()){
        var target = result.getBestTarget();
        double distance = PhotonUtils.calculateDistanceToTargetMeters(robotToCamera.getZ(), Vision_Constants.noteDetectionHeight, robotToCamera.getRotation().getY(), Units.degreesToRadians(target.getPitch()));
        latestMetersToNote = distance;
        return distance;
      
      }
      return latestMetersToNote;
    }
    

    public Translation2d getTranslationToNote(){
      Transform3d robotToCamera = Vision_Constants.objectCameraToRobot;
      var result = objectCamera.getLatestResult();
      if(result.hasTargets()){
        var target = result.getBestTarget();
        double distance = PhotonUtils.calculateDistanceToTargetMeters(robotToCamera.getZ(), Vision_Constants.noteDetectionHeight, robotToCamera.getRotation().getY(), Units.degreesToRadians(target.getPitch()));
        Translation2d translation = PhotonUtils.estimateCameraToTargetTranslation(distance, Rotation2d.fromDegrees(-target.getYaw()));
        latestTranslationToNote = translation;
        return translation;
      
      }
      return latestTranslationToNote;
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Yaw to Object (Deg)", getTranslationToNote().getAngle().getDegrees());
        SmartDashboard.putNumber("Distance to Object (m)", getMetersToNote());

    }
}
