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

    final int encodersPerRev = 42; // encoder counts per revolution of the motor
    final double gearRatio = 12.75; // motor spins 12.75 times for wheel to spin once

    //driveTrain

    //private SparkMaxPIDController flEncoder = new SparkMaxPIDController(flMotor)

    private double initialAngle;

    MecanumDrive mecanumDrive = new MecanumDrive(flMotor, blMotor, frMotor, brMotor);

    public void init(){
        frMotor.setInverted(true);
        brMotor.setInverted(true);

        flMotor.getEncoder().setPosition(0);
        frMotor.getEncoder().setPosition(0);
        blMotor.getEncoder().setPosition(0);
        brMotor.getEncoder().setPosition(0);
    }

    public void driveByInches(double inches){
        
        
    }

    

}



