package frc.robot;


public class Vision {
    
    private static double cameraHeight = 26.5;
    private static double cameraAngle = 0.0;

    public static double getDistancefromY(double verticalAngle, double targetHeight)
    {
        return (targetHeight - cameraHeight) / Math.tan(verticalAngle + cameraAngle);
    }
}
