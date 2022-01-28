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

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {

  public Drivetrain drive = new Drivetrain();

  private final Joystick xbox = new Joystick(1);
  // WPI_TalonSRX wpi_TalonSRX = new WPI_TalonSRX(1);

  NetworkTable limelight;
  Limelight forwardCamera;
  
  QuadraticController strafeController = new QuadraticController(0.1, 0.15);
  
  @Override
  public void robotInit() {

    //initialize networktables instance, grab the limelight table
    NetworkTableInstance table = NetworkTableInstance.getDefault();
    limelight = table.getTable("limelight");
    
    
  }

  @Override
  public void autonomousInit() {
    //initialize the camera withe the table
    forwardCamera = new Limelight(limelight);
  }

  @Override
  public void autonomousPeriodic() {
    if(forwardCamera.visible())
    {
      System.out.println("angleOffset: " + forwardCamera.angleOffsetX());
      

      double encoderCounts = forwardCamera.angleOffsetX();

      double speed = strafeController.quadraticTurn(encoderCounts, forwardCamera.angleOffsetX());
      
      //strafe with the calculated speed towards the ball
      if(forwardCamera.angleOffsetX() < -5.0){
        drive.mecanumDrive.driveCartesian(0,speed,0);
      } else if(forwardCamera.angleOffsetX() > 5.0) {
        drive.mecanumDrive.driveCartesian(0,-speed,0);
      }
      
      //drive.flMotor.set(0.5);
      System.out.println("I see the target");
    } else{
      //drive.flMotor.set(0.0);
      System.out.println("I do not see the target");
    }
  }
  @Override
  public void teleopInit() {
    drive.init();

    drive.flMotor.getEncoder().setPositionConversionFactor(42);
    drive.frMotor.getEncoder().setPositionConversionFactor(42);
    drive.blMotor.getEncoder().setPositionConversionFactor(42);
    drive.brMotor.getEncoder().setPositionConversionFactor(42);
    
    //drive.flMotor.set(.2);
  }

  @Override
  public void teleopPeriodic() {
    //System.out.println(WPILibVersion.Version);
    
    // flMotor.set(xbox.getY() * 0.87);
    // frMotor.set(-xbox.getY() * 0.87);
    // blMotor.set(xbox.getY() * 0.87);
    // brMotor.set(-xbox.getY() * 0.87);

    //System.out.println(drive.flMotor.getEncoder().getPosition());


    double ySpeed = xbox.getRawAxis(1);
    double xSpeed = xbox.getRawAxis(0);
    double zSpeed = xbox.getRawAxis(2);

    if((ySpeed > -0.20) && (ySpeed < 0.20)){
      ySpeed = 0.00;
    }

    if((xSpeed > -0.20) && (xSpeed < 0.20)){
      xSpeed = 0.00;
    }

    if((zSpeed > -0.20) && (zSpeed < 0.20)){
      zSpeed = 0.00;
    }

    if(ySpeed > 0.87){
      ySpeed = 0.87;
    }

    if(xSpeed > 0.87){
      xSpeed = 0.87;
    }

    if(zSpeed > 0.87){
      zSpeed = 0.87;
    }

    drive.mecanumDrive.driveCartesian(-ySpeed, xSpeed, zSpeed);

    double encoderCounts = 535.5*4;
    double minimumMotorSpeed = 0.05; // 0.011 for 250
    double maximumMotorSpeed = 0.3;

    System.out.println(drive.flMotor.getEncoder().getPosition());
    System.out.println(drive.frMotor.getEncoder().getPosition());
    System.out.println(drive.blMotor.getEncoder().getPosition());
    System.out.println(drive.brMotor.getEncoder().getPosition());
    System.out.println("*******************************************");

    double a = ((encoderCounts * maximumMotorSpeed - minimumMotorSpeed * encoderCounts
      - minimumMotorSpeed * (encoderCounts / 2) + minimumMotorSpeed * (encoderCounts / 2))
      / (encoderCounts * (encoderCounts / 2) * ((encoderCounts / 2) - encoderCounts)));

    double b = ((maximumMotorSpeed - a * (encoderCounts / 2) * (encoderCounts / 2) - minimumMotorSpeed) / (encoderCounts / 2));

    double flspeed = a * drive.flMotor.getEncoder().getPosition() * drive.flMotor.getEncoder().getPosition()
    + b * drive.flMotor.getEncoder().getPosition() + minimumMotorSpeed;

    double frspeed = a * drive.frMotor.getEncoder().getPosition() * drive.frMotor.getEncoder().getPosition()
    + b * drive.frMotor.getEncoder().getPosition() + minimumMotorSpeed;

    double blspeed = a * drive.blMotor.getEncoder().getPosition() * drive.blMotor.getEncoder().getPosition()
    + b * drive.blMotor.getEncoder().getPosition() + minimumMotorSpeed;

    double brspeed = a * drive.brMotor.getEncoder().getPosition() * drive.brMotor.getEncoder().getPosition()
    + b * drive.brMotor.getEncoder().getPosition() + minimumMotorSpeed;

    
    drive.flMotor.set(flspeed);
    drive.frMotor.set(frspeed);
    drive.blMotor.set(blspeed);
    drive.brMotor.set(brspeed);
    

    
    
    // if(drive.flMotor.getEncoder().getPosition() >= 250.0){
    //   drive.flMotor.set(0.0);
    // }

    


    // drive.flMotor.getEncoder().setPosition(20000);
    // drive.frMotor.getEncoder().setPosition(20000);
    // drive.blMotor.getEncoder().setPosition(20000);
    // drive.brMotor.getEncoder().setPosition(20000);
    //drive.flMotor.getAlternateEncoder(Type.kQuadrature, 8192).setPosition(250);
    //drive.brMotor.getEncoder().set

    

    
    }
    
  

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}
}