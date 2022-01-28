package frc.robot;

public class QuadraticController {

    //minimum and maximum motor speeds

    private double minMotSpeed;
    private double maxMotSpeed;

    public QuadraticController()
    {
        minMotSpeed = 0.1;
        maxMotSpeed = 0.19;
    }

    public QuadraticController(double min, double max)
    {
        minMotSpeed = min;
        maxMotSpeed = max;
    }

    public double quadraticTurn(double angleToTurnInitial, double angleToTurnCurrent)
    {


        double minimumMotorSpeed = minMotSpeed;
        double maximumMotorSpeed = maxMotSpeed;
        double encoderCounts = angleToTurnInitial;
    
    
        double a = ((encoderCounts * maximumMotorSpeed - minimumMotorSpeed * encoderCounts
          - minimumMotorSpeed * (encoderCounts / 2) + minimumMotorSpeed * (encoderCounts / 2))
          / (encoderCounts * (encoderCounts / 2) * ((encoderCounts / 2) - encoderCounts)));
    
        double b = ((maximumMotorSpeed - a * (encoderCounts / 2) * (encoderCounts / 2) - minimumMotorSpeed) / (encoderCounts / 2));
    
        double speed = a * angleToTurnCurrent * angleToTurnCurrent
        + b * angleToTurnCurrent + minimumMotorSpeed;
    
        return speed;
    }
}
