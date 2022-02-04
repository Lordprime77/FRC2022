// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

import javax.sound.sampled.SourceDataLine;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.*;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
//import edu.wpi.first.wpilibj.
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder.Type;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.util.WPILibVersion;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import edu.wpi.first.wpilibj.util.Color;



public class Robot extends TimedRobot {

  
  CvSource outputStream;

  Timer timer = new Timer();

  public Drivetrain drive = new Drivetrain();


  static Joystick stick;
  static Joystick aux;

  //Sensitivity of inputs
  double xSensitivity = 0.5;
  double ySensitivity = 0.5;
  double zSensitivity = 0.3;
  
  //disable for testing purposes, or if we need auto in teleop
  boolean manualControlEnabled = true;

  //Joystick is 1, xbox controller is 3
  int stickID = 1;
  int auxID = 3;


  int stage = 1;

  //probably useless
  int movestage;

  //for testing purposes, might use in actual code
  boolean shouldBeDriving;

  //enables "drive until the color sensor sees a color" mode. Here because Mr. Lathrop thinks we need it
  boolean colorTestMode = false;

  NetworkTable limelight;
  Limelight forwardCamera;

  //variables for a limit switch (for testing purposes)
  DigitalInput limitSwitch;
  int limitSwitchPort = 0;
  
  QuadraticController strafeController = new QuadraticController(0.1, 0.15);

  //color sensor on i2c
  private final I2C.Port port = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(port);
  private final ColorMatch matcher = new ColorMatch();
  private final Color blueTarget = new Color(0.143, 0.427, 0.429);
  private final Color redTarget = new Color(0.561, 0.232, 0.114);
  private final Color defaultColor = new Color(0.336, 0.470,0.195);


  Color detectedColor;
  
  @Override
  public void robotInit() {

    CameraServer.startAutomaticCapture();
    

    //initialize networktables instance, grab the limelight table
    NetworkTableInstance table = NetworkTableInstance.getDefault();
    limelight = table.getTable("limelight");

    
    stick = new Joystick(stickID);
    aux = new Joystick(auxID);

    
    //initialize limit switch
    limitSwitch = new DigitalInput(limitSwitchPort);

    //Add coolor to matching algorithm
    matcher.addColorMatch(blueTarget);
    matcher.addColorMatch(redTarget);
    matcher.addColorMatch(defaultColor);

    shouldBeDriving = true;
  }

  @Override
  public void autonomousInit() {
    drive.init();
    timer.reset();
    timer.start();
    stage = 1;
    movestage = 0;
    //initialize the camera withe the table
    forwardCamera = new Limelight(limelight);
  }

  @Override
  public void autonomousPeriodic() {

    /*
    System.out.println("stage: " + stage);
    System.out.println("timer: " + timer.get() + " seconds");
    System.out.println("fl motor: " + drive.flMotor.getEncoder().getPosition());
    //System.out.println(drive.flMotor.get());



      switch(stage) {
      case 1 : {
        drive.driveTrainByInches(50);
        if(timer.get() > 4.0){
          drive.resetDriveTrainEncoders();
          stage = 2;
        }
        break;
      }
      case 2 : {
        drive.driveTrainByInches(-100.0);
        if(timer.get() > 15.0){
          drive.resetDriveTrainEncoders();
          stage = 3;
        }
        break;
      }
      case 3 : {
        drive.driveTrainByInches(50);
        if(timer.get() > 20.0){
          drive.resetDriveTrainEncoders();
          stage = 4;
        }
        break;
      }
  }
  */

  //drive if not limit switch
  /*
 if(!limitSwitch.get())
 {
   drive.driveTrainTeleop(0, 0.21, 0);
 } else{
   drive.driveTrainTeleop(0, 0, 0);
 }
*/

 detectedColor = m_colorSensor.getColor();
 ColorMatchResult match = matcher.matchClosestColor(detectedColor);
 if(match.color == blueTarget)
 {
   System.out.println("blue");
   shouldBeDriving = false;
 } else if(match.color == redTarget)
 {
   System.out.println("red");
   shouldBeDriving = false;
 }else if (match.color == defaultColor)
 {
  System.out.println("Nothing");
 } else {
   System.out.println("Unknown");
 }





 switch(movestage)
 {
   case 0 :
   {
     drive.resetDriveTrainEncoders();
     timer.reset();
     movestage = 1;
   }

   case 1 :
   {
     drive.driveTrainByInches(25);
     if(!drive.isMoving() && timer.get() > 1.0)
     {
       movestage = 2;
     }
   }

   case 2 :
   {
     shouldBeDriving = false;
   }
 }



 shouldBeDriving = true;

 


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
  }

  @Override
  public void teleopPeriodic() {

    

    if(manualControlEnabled && stickID == 1){
      drive.driveTrainTeleop( stick.getRawAxis(0) * xSensitivity,-1 * stick.getRawAxis(1) * ySensitivity, stick.getRawAxis(2) * zSensitivity);
    } else if( manualControlEnabled && stickID == 3)
    {
      drive.driveTrainTeleop( stick.getRawAxis(0) * xSensitivity,-1 * stick.getRawAxis(1) * ySensitivity, stick.getRawAxis(4) * zSensitivity);
    }

    

    
    }
    
  

  @Override
  public void testInit() {
    
  }

  @Override
  public void testPeriodic() {}
}