// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

import javax.sound.sampled.SourceDataLine;

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

public class Robot extends TimedRobot {

  Timer timer = new Timer();

  public Drivetrain drive = new Drivetrain();


  static Joystick stick;

  //Sensitivity of inputs
  double xSensitivity = 0.5;
  double ySensitivity = 0.5;
  double zSensitivity = 0.3;
  
  //If the controller is not a joystick, it defaults to an XBOX controller
  boolean controllerIsJoystick = false;
  boolean manualControlEnabled = true;
  int stage = 1;


  NetworkTable limelight;
  Limelight forwardCamera;

  //variables for a limit switch (for testing purposes)
  DigitalInput limitSwitch;
  int limitSwitchPort = 0;
  
  QuadraticController strafeController = new QuadraticController(0.1, 0.15);
  
  @Override
  public void robotInit() {
    

    //initialize networktables instance, grab the limelight table
    NetworkTableInstance table = NetworkTableInstance.getDefault();
    limelight = table.getTable("limelight");

    if(manualControlEnabled && controllerIsJoystick)
    {
      stick = new Joystick(1);
    } else if(manualControlEnabled && !controllerIsJoystick)
    {
      stick = new Joystick(3);
    }
    
    //initialize limit switch
    limitSwitch = new DigitalInput(limitSwitchPort);
  }

  @Override
  public void autonomousInit() {
    drive.init();
    timer.reset();
    timer.start();
    stage = 1;
    //initialize the camera withe the table
    forwardCamera = new Limelight(limelight);
  }

  @Override
  public void autonomousPeriodic() {
    System.out.println("stage: " + stage);
    System.out.println("timer: " + timer.get() + " seconds");
    System.out.println("fl motor: " + drive.flMotor.getEncoder().getPosition());
    //System.out.println(drive.flMotor.get());


/*
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
 if(!limitSwitch.get())
 {
   drive.driveTrainTeleop(0, 0.21, 0);
 } else{
   drive.driveTrainTeleop(0, 0, 0);
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
  }

  @Override
  public void teleopPeriodic() {

    

    if(controllerIsJoystick && manualControlEnabled){
      drive.driveTrainTeleop( stick.getRawAxis(0) * xSensitivity,-1 * stick.getRawAxis(1) * ySensitivity, stick.getRawAxis(2) * zSensitivity);
    } else if(!controllerIsJoystick && manualControlEnabled) 
    {
      drive.driveTrainTeleop(stick.getRawAxis(0) * xSensitivity,-1 * stick.getRawAxis(1) * ySensitivity,  stick.getRawAxis(4) * zSensitivity);
    }

    

    
    }
    
  

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}
}