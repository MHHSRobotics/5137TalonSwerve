package frc.robot.Constants;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class Vision_Constants {

    public static final double objectCameraX = Units.inchesToMeters(0);
    public static final double objectCameraY = 0.0;
    public static final double objectCameraZ = Units.inchesToMeters(10);
    public static final double objectCameraRoll = 0.0;
    public static final double objectCameraPitch = Math.toRadians(-5);
    public static final double objectCameraYaw = 0.0;
    public static final Transform3d objectCameraToRobot = new Transform3d(objectCameraX, objectCameraY, objectCameraZ, new Rotation3d(objectCameraRoll, objectCameraPitch, objectCameraYaw));

    public static final double noteDetectionHeight = 0.0; 
    public static final double notePickupDistance = Units.inchesToMeters(0.0);

    public static final double objectDistanceKP = 1.0; 
    public static final double objectDistanceKI = 0.0; 
    public static final double objectDistanceKD = 0.0; 

    public static final double objectYawKP = 2.0; 
    public static final double objectYawKI = 0.0; 
    public static final double objectYawKD = 0.0; 

    //TODO: Change for real robot
    public static final double allowableDistanceError = .65;
    public static final double allowableRotationError = Units.degreesToRadians(3);


}
