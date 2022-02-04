// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

import javax.sound.sampled.SourceDataLine;

//import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
//import edu.wpi.first.wpilibj.
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder.Type;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.util.WPILibVersion;

public class Robot extends TimedRobot {

  Timer autoTimer = new Timer();

  public Drivetrain drive = new Drivetrain();

  static Joystick xbox = new Joystick(1);

  int stage = 1;

  NetworkTable limelight;
  Limelight forwardCamera;
  
  @Override
  public void robotInit() {
    

    //initialize networktables instance, grab the limelight table
    NetworkTableInstance table = NetworkTableInstance.getDefault();
    limelight = table.getTable("limelight");
    
    
  }

  @Override
  public void autonomousInit() {
    drive.init();
    autoTimer.reset();
    autoTimer.start();
    stage = 1;
    //initialize the camera withe the table
    forwardCamera = new Limelight(limelight);
  }

  @Override
  public void autonomousPeriodic() {
    System.out.println("stage: " + stage);
    System.out.println("timer: " + autoTimer.get() + " seconds");
    System.out.println("fl motor: " + drive.flMotor.getEncoder().getPosition());

      switch(stage) {
      case 1 : {
        drive.driveTrainByInches(50.0, 0);
        if(autoTimer.get() > 4.0){
          drive.resetDriveTrainEncoders();
          stage = 2;
        }
        break;
      }
      case 2 : {
        drive.driveTrainByInches(100.0, 1);
        if(autoTimer.get() > 15.0){
          drive.resetDriveTrainEncoders();
          stage = 3;
        }
        break;
      }
      case 3 : {
        drive.driveTrainByInches(50.0, 0);
        if(autoTimer.get() > 20.0){
          drive.resetDriveTrainEncoders();
          stage = 4;
        }
        break;
      }
  }
}
    



    // if(forwardCamera.visible())
    // {
    //   System.out.println("angleOffset: " + forwardCamera.angleOffsetX());

    //   double speed = strafeController.quadraticPositionAndSpeed(forwardCamera.angleOffsetX(), forwardCamera.angleOffsetX());
      
    //   //strafe with the calculated speed towards the ball
    //   if(forwardCamera.angleOffsetX() < -3.0){
    //     drive.mecanumDrive.driveCartesian(0,speed,0);
    //   } else if(forwardCamera.angleOffsetX() > 3.0) {
    //     drive.mecanumDrive.driveCartesian(0,-speed,0);
    //   }
      
    //   System.out.println("I see the target");
    // } else{
    //   System.out.println("I do not see the target");
    // }
  

  @Override
  public void teleopInit() {
    drive.init();
    drive.resetDriveTrainEncoders();
    drive.turningTimer.start();
  }

  @Override
  public void teleopPeriodic() {
    

    if(!drive.turningL90 && !drive.turningR90){
      //drive.driveTrainTeleop();
    }
    // System.out.println(drive.inchesToEncoders(27));
    // System.out.println(drive.encodersToInches(drive.inchesToEncoders(27)));
    System.out.println("FL position: " + drive.flMotor.getEncoder().getPosition());
    System.out.println("FR position: " + drive.frMotor.getEncoder().getPosition());
    System.out.println("BL position: " + drive.blMotor.getEncoder().getPosition());
    System.out.println("BR position: " + drive.brMotor.getEncoder().getPosition());
    System.out.println("Timer: " + drive.turningTimer.get() + " s");

    if(this.xbox.getRawButton(1) && drive.button1Boolean){
      if(drive.haventresetEncodersYet){
        drive.resetDriveTrainEncoders();
        drive.haventresetEncodersYet = false;
      }
      
      drive.turningL90 = true;
      drive.turnGoal += drive.ninetyDegreeTurnInches;
      drive.turningTimer.reset();
    }
    /////////////////////////////////////////////////////////////////////////////////////////////////////

    if(this.xbox.getRawButton(2) && drive.button2Boolean){
      if(drive.haventresetEncodersYet){
        drive.resetDriveTrainEncoders();
        drive.haventresetEncodersYet = false;
      }

      drive.turningR90 = true;
      drive.turnGoal += drive.ninetyDegreeTurnInches;
      drive.turningTimer.reset();
    }
    //////////////////////////////////////////////////////////////////////////////////////////////////////

    if(drive.turningL90){
      drive.driveTrainByInches(drive.turnGoal, 4);

      if(drive.turningTimer.get() > 1.4){ // need to create variables for how long a turn is supposed to take
        drive.resetDriveTrainEncoders();
        drive.turningL90 = false;
        drive.haventresetEncodersYet = true;
        drive.turnGoal = 0.0;
        }
    }
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////

    if(drive.turningR90){
      drive.driveTrainByInches(drive.turnGoal, 5);

      if(drive.turningTimer.get() > 1.4){ // need to create variables for how long a turn is supposed to take
        drive.resetDriveTrainEncoders();
        drive.turningR90 = false;
        drive.haventresetEncodersYet = true;
        drive.turnGoal = 0.0;
      }
    }


    // System.out.println("timer: " + drive.turningTimer.get());
    // System.out.println("turngoal: " + drive.turnGoal);
    // System.out.println("turningEncoders: " + drive.turningEncoders);

    // if(this.xbox.getRawButton(1) && drive.button1Boolean){
    //   // if(drive.haventresetEncodersYet){
    //   //   drive.resetDriveTrainEncoders();
    //   //   drive.haventresetEncodersYet = false;
    //   // }
    //   drive.resetDriveTrainEncoders();
    //   drive.turningL90 = true;
    //   //drive.turnGoal += drive.ninetyDegreeTurnInches;
    //   drive.turnGoal = drive.ninetyDegreeTurnInches;
    //   //drive.turnGoal += 2;
    //   drive.turningTimer.reset();
    // }
    // /////////////////////////////////////////////////////////////////////////////////////////////////////

    // if(this.xbox.getRawButton(2) && drive.button2Boolean){
    //   //drive.turningEncoders = drive.flMotor.getEncoder().getPosition();
    //   // if(drive.turningL90){
    //   //   drive.resetDriveTrainEncoders();
    //   // }
    //   // if(drive.haventresetEncodersYet){
    //   //   drive.resetDriveTrainEncoders();
    //   //   drive.haventresetEncodersYet = false;
    //   // }
    //   drive.resetDriveTrainEncoders();
    //   drive.turningR90 = true;
    //   //drive.turnGoal += drive.ninetyDegreeTurnInches;
    //   // drive.turnGoal = drive.ninetyDegreeTurnInches;
    //   drive.turnGoal = drive.ninetyDegreeTurnInches;
    //   drive.turningTimer.reset();
    // }
    // //////////////////////////////////////////////////////////////////////////////////////////////////////

    // if(drive.turningL90){
    //   drive.driveTrainByInches(drive.turnGoal, 4);

    //   if(drive.turningTimer.get() > 1.0){ // need to create variables for how long a turn is supposed to take
    //     drive.resetDriveTrainEncoders();
    //     drive.turningL90 = false;
    //     //drive.haventresetEncodersYet = true;
    //     drive.turnGoal = 0.0;
    //     }

    //   // drive.mecanumDrive.driveCartesian(0.0, 0.0, this.quadraticPositionAndSpeed(0.1, 0.5, , currentPosition));
    //   // this.quadraticPositionAndSpeed(0.1, 0.5, positionGoal, currentPosition)
    //   // if(!drive.turningR90){
        
    //   // }
    // }
    // //////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // if(drive.turningR90){
    //   drive.driveTrainByInches(drive.turnGoal, 5);

    //   if(drive.turningTimer.get() > 1.0){ // need to create variables for how long a turn is supposed to take
    //     drive.resetDriveTrainEncoders();
    //     drive.turningR90 = false;
    //     //drive.haventresetEncodersYet = true;
    //     drive.turnGoal = 0.0;
    //   }
    //   // if(drive.turningL90){
    //   //   drive.driveTrainByInches(drive.encodersToInches(drive.turningEncoders), 5);
    //   // }
    //   // if(!drive.turningL90){
        
    //   // }
    // }
      /////////////////////////////////////////////////////////////////////////////////////////////////////

    

    

    


    

    // if(controllerIsJoystick && manualControlEnabled){
    //   drive.driveTrainTeleop( stick.getRawAxis(0) * xSensitivity,-1 * stick.getRawAxis(1) * ySensitivity, stick.getRawAxis(2) * zSensitivity);
    // } else if(!controllerIsJoystick && manualControlEnabled) 
    // {
    //   drive.driveTrainTeleop(stick.getRawAxis(0) * xSensitivity,-1 * stick.getRawAxis(1) * ySensitivity,  stick.getRawAxis(4) * zSensitivity);
    // }

    

    




    

    if(xbox.getRawButton(1)){
      drive.button1Boolean = false;
    } else {
      drive.button1Boolean = true;
    }

    if(xbox.getRawButton(2)){
      drive.button2Boolean = false;
    } else {
      drive.button2Boolean = true;
    }
  }
    
  

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  public static double quadraticPositionAndSpeed(double minimumMotorSpeed, double maximumMotorSpeed, double positionGoal, double currentPosition) {

    // position could be angle offset, encoder count, inches, etc.

    // maximum speed should be reached at the middle position

    // need to look at system of equations again and see if I want endpoint to just be positionGoal with speed 0. (position, 0)

    double a = ((positionGoal * maximumMotorSpeed - minimumMotorSpeed * positionGoal - minimumMotorSpeed * (positionGoal / 2) + minimumMotorSpeed * (positionGoal / 2)) / (positionGoal * (positionGoal / 2) * ((positionGoal / 2) - positionGoal)));

    double b = ((maximumMotorSpeed - a * (positionGoal / 2) * (positionGoal / 2) - minimumMotorSpeed) / (positionGoal / 2));

    double speed = a * currentPosition * currentPosition + b * currentPosition + minimumMotorSpeed;
    
    return speed; // value would be from 0.0 to 1.0 like any motor needs to be
  }

}

