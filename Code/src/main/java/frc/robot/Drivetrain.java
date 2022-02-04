package frc.robot;
import com.ctre.phoenix.CANifier.GeneralPin;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;

public class Drivetrain {

    CANSparkMax flMotor = new CANSparkMax(4, MotorType.kBrushless);
    CANSparkMax frMotor = new CANSparkMax(6, MotorType.kBrushless);
    CANSparkMax blMotor = new CANSparkMax(7, MotorType.kBrushless);
    CANSparkMax brMotor = new CANSparkMax(3, MotorType.kBrushless);




    // making all doubles in case of weird mathematic things happening
    final static double ENCODERS_PER_REV = 42.0; // encoder counts per revolution of the motor
    final static double GEAR_RATIO = 12.75; // inches // motor spins 12.75 times for wheel to spin once
    final static double wheelRadius = 4.0; // inches
    final static double driveTrainInchesOffset = 1.5; // inches

    final double minMotorSpeedEncoders = 0.1; // need to test these numbers for best accuracy
    final double maxMotorSpeedEncoders = 0.4; // may change with robot weight

    final double motorSpeedThresholdTeleop = 0.2;
    final double maxMotorSpeedTeleop = 0.87;


    // TURNING with encoders
    public boolean turningR90 = false;
    public boolean turningL90 = false;
    public boolean turning180 = false;
    public double turningEncoders = 0.0;
    public Timer turningTimer = new Timer();
    public static double xDistancBetweenWheels = 20.5;
    public static double yDistanceBetweenWheels = 21.5;
    public static double ninetyDegreeTurnInches = ((Math.sqrt(Math.pow(20.5,2)+Math.pow(21.5,2)) * Math.PI) / 4);
    public double turnGoal = 0.0;
    public boolean haventresetEncodersYet = true;
    public boolean button1Boolean = true;
    public boolean button2Boolean = true;

    

    MecanumDrive mecanumDrive = new MecanumDrive(flMotor, blMotor, frMotor, brMotor);

    public void init(){
        frMotor.setInverted(true);
        brMotor.setInverted(true);

        resetDriveTrainEncoders();
        
        flMotor.getEncoder().setPositionConversionFactor(ENCODERS_PER_REV);
        frMotor.getEncoder().setPositionConversionFactor(ENCODERS_PER_REV);
        blMotor.getEncoder().setPositionConversionFactor(ENCODERS_PER_REV);
        brMotor.getEncoder().setPositionConversionFactor(ENCODERS_PER_REV);
    }

    public void driveTrainTeleop() {

        double ySpeed = Robot.xbox.getRawAxis(0);
        double xSpeed = Robot.xbox.getRawAxis(1);
        double zSpeed = Robot.xbox.getRawAxis(2);

        if((ySpeed > -motorSpeedThresholdTeleop) && (ySpeed < motorSpeedThresholdTeleop)) { // minimum thresholds to go so no interference
            ySpeed = 0.00;
        }

        if((xSpeed > -motorSpeedThresholdTeleop) && (xSpeed < motorSpeedThresholdTeleop)) {
            xSpeed = 0.00;
        }

        if((zSpeed > -motorSpeedThresholdTeleop) && (zSpeed < motorSpeedThresholdTeleop)) {
            zSpeed = 0.00;
        }

        if(ySpeed > maxMotorSpeedTeleop) {
           ySpeed = maxMotorSpeedTeleop;
        }

        if(xSpeed > maxMotorSpeedTeleop) {
            xSpeed = maxMotorSpeedTeleop;
        }

        if(zSpeed > maxMotorSpeedTeleop) {
            zSpeed = maxMotorSpeedTeleop;
        }

        mecanumDrive.driveCartesian(-ySpeed, xSpeed, zSpeed);
    }


    public void driveTrainByInches(double inches, int direction){ // 0 = forward. 1 = back. 2 = left. 3 = right. 4 = turn left. 5 = turn right
        
        //double encoderCounts = 0.0;
        // minus 1.5 (driveTrainInchesOffset) will have to be test for accuracy on different speeds and when robot weight changes
        // we could literally make the encoderCount PositionConversionFactor equate to inches outright. That could
        // just be for the drive train

        if(direction == 0){ // forward
            mecanumDrive.driveCartesian(Robot.quadraticPositionAndSpeed(minMotorSpeedEncoders, 
            maxMotorSpeedEncoders, inchesToEncoders(inches), flMotor.getEncoder().getPosition()), 0.0, 0.0);
        }

        if(direction == 1){ // back
            mecanumDrive.driveCartesian(-Robot.quadraticPositionAndSpeed(minMotorSpeedEncoders, 
            maxMotorSpeedEncoders, -inchesToEncoders(inches), flMotor.getEncoder().getPosition()), 0.0, 0.0);
        }

        if(direction == 2){ // left
            mecanumDrive.driveCartesian(0.0, Robot.quadraticPositionAndSpeed(minMotorSpeedEncoders, 
            maxMotorSpeedEncoders, inchesToEncoders(inches), flMotor.getEncoder().getPosition()), 0.0);
        }

        if(direction == 3){ // right
            mecanumDrive.driveCartesian(0.0, -Robot.quadraticPositionAndSpeed(minMotorSpeedEncoders, 
            maxMotorSpeedEncoders, -inchesToEncoders(inches), flMotor.getEncoder().getPosition()), 0.0);
        }

        if(direction == 4){ // turn left
            mecanumDrive.driveCartesian(0.0, 0.0, Robot.quadraticPositionAndSpeed(minMotorSpeedEncoders, 
            maxMotorSpeedEncoders, inchesToEncoders(inches), flMotor.getEncoder().getPosition()));
        }

        if(direction == 5){ // turn right
            mecanumDrive.driveCartesian(0.0, 0.0, -Robot.quadraticPositionAndSpeed(minMotorSpeedEncoders, 
            maxMotorSpeedEncoders, -inchesToEncoders(inches), flMotor.getEncoder().getPosition()));
        }




    }

    public void turn90Left() {

    }

    public void turn90Right() {

    }

    public void turn180() {
        
    }

    
    public void resetDriveTrainEncoders() {
        flMotor.getEncoder().setPosition(0);
        frMotor.getEncoder().setPosition(0);
        blMotor.getEncoder().setPosition(0);
        brMotor.getEncoder().setPosition(0);
    }

    public static double inchesToEncoders(double inches){
        return ((   (Math.abs(inches) - driveTrainInchesOffset)  /  (2.0*Math.PI*wheelRadius)  ) * ENCODERS_PER_REV * GEAR_RATIO);
      }

      public static double encodersToInches(double encoders){
        //   encoders /= GEAR_RATIO;
        //   encoders /= ENCODERS_PER_REV;
        //   encoders *= 2.0*Math.PI*wheelRadius;
        //   encoders = Math.abs(encoders);
        //   encoders +=driveTrainInchesOffset;
        //   return encoders;
          return ((((encoders / GEAR_RATIO) / ENCODERS_PER_REV) * (2.0*Math.PI*wheelRadius)) + driveTrainInchesOffset);
      }
}



