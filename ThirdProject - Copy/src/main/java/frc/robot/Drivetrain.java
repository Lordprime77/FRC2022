package frc.robot;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

public class Drivetrain {

    CANSparkMax flMotor = new CANSparkMax(4, MotorType.kBrushless);
    CANSparkMax frMotor = new CANSparkMax(6, MotorType.kBrushless);
    CANSparkMax blMotor = new CANSparkMax(7, MotorType.kBrushless);
    CANSparkMax brMotor = new CANSparkMax(3, MotorType.kBrushless);




    // making all doubles in case of weird mathematic things happening
    final double ENCODERS_PER_REV = 42.0; // encoder counts per revolution of the motor
    final double GEAR_RATIO = 12.75; // inches // motor spins 12.75 times for wheel to spin once
    final double wheelRadius = 4.0; // inches
    final double driveTrainInchesOffset = 1.5; // inches

    final double minMotorSpeedEncoders = 0.1; // need to test these numbers for best accuracy
    final double maxMotorSpeedEncoders = 0.4; // may change with robot weight

    final double motorSpeedThresholdTeleop = 0.2;
    final double maxMotorSpeedTeleop = 0.87;

    final double encoderCount = ((25 - driveTrainInchesOffset)/(2.0*Math.PI*wheelRadius)) * ENCODERS_PER_REV * GEAR_RATIO;

    QuadraticController driveTrainQuadraticController = new QuadraticController(minMotorSpeedEncoders, maxMotorSpeedEncoders); 
    

    MecanumDrive mecanumDrive = new MecanumDrive(flMotor, blMotor, frMotor, brMotor);

    public void init(){
        System.out.println("init called!");
        //System.out.println(((25 - driveTrainInchesOffset)/(2.0*Math.PI*wheelRadius)) * ENCODERS_PER_REV * GEAR_RATIO);
        frMotor.setInverted(true);
        brMotor.setInverted(true);

        resetDriveTrainEncoders();
        
        flMotor.getEncoder().setPositionConversionFactor(ENCODERS_PER_REV);
        frMotor.getEncoder().setPositionConversionFactor(ENCODERS_PER_REV);
        blMotor.getEncoder().setPositionConversionFactor(ENCODERS_PER_REV);
        brMotor.getEncoder().setPositionConversionFactor(ENCODERS_PER_REV);
    }

    public void driveTrainTeleop(double xInput, double yInput, double zInput) {

        //Changed the method slightly so it can take input from any controller setup
        double ySpeed = yInput;
        double xSpeed = xInput;
        double zSpeed = zInput;

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


    public void driveTrainByInches(double inches){
        
        double encoderCounts = Math.abs(((inches - driveTrainInchesOffset)/(2.0*Math.PI*wheelRadius)) * ENCODERS_PER_REV * GEAR_RATIO);
        //double encoderCounts = 0.0;
        // minus 1.5 will have to be test for accuracy on different speeds and when robot weight changes
        // we could literally make the encoderCount PositionConversionFactor equate to inches outright. That could
        // just be for the drive train

        
        if (inches > 0.0) {
            flMotor.set(driveTrainQuadraticController.quadraticPositionAndSpeed(encoderCounts, flMotor.getEncoder().getPosition()));
            frMotor.set(driveTrainQuadraticController.quadraticPositionAndSpeed(encoderCounts, frMotor.getEncoder().getPosition()));
            blMotor.set(driveTrainQuadraticController.quadraticPositionAndSpeed(encoderCounts, blMotor.getEncoder().getPosition()));
            brMotor.set(driveTrainQuadraticController.quadraticPositionAndSpeed(encoderCounts, brMotor.getEncoder().getPosition()));
        } else {
            flMotor.set(-driveTrainQuadraticController.quadraticPositionAndSpeed(-encoderCounts, flMotor.getEncoder().getPosition()));
            frMotor.set(-driveTrainQuadraticController.quadraticPositionAndSpeed(-encoderCounts, frMotor.getEncoder().getPosition()));
            blMotor.set(-driveTrainQuadraticController.quadraticPositionAndSpeed(-encoderCounts, blMotor.getEncoder().getPosition()));
            brMotor.set(-driveTrainQuadraticController.quadraticPositionAndSpeed(-encoderCounts, brMotor.getEncoder().getPosition()));
        }
        

        // if(flMotor.get() < minMotorSpeedEncoders){
        //     return true;
        // }

        // if(flMotor.getEncoder().getPosition() < encoderCounts){
        //     return true;
        // }

        // return false;
    }

    public double inchesToEncoders(double inches) {
        return Math.abs(((inches - driveTrainInchesOffset)/(2.0*Math.PI*wheelRadius)) * ENCODERS_PER_REV * GEAR_RATIO);
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

    public boolean isMoving()
    {
        return flMotor.getEncoder().getVelocity() > 0.05 && frMotor.getEncoder().getVelocity() > 0.05 && blMotor.getEncoder().getVelocity() > 0.05 && brMotor.getEncoder().getVelocity() > 0.05;
    }
}



