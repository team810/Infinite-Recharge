/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  public final CANSparkMax front_L = new CANSparkMax(Constants.FRONTL, MotorType.kBrushless);
  public final CANSparkMax front_R = new CANSparkMax(Constants.FRONTR, MotorType.kBrushless);
  public final CANSparkMax back_L =  new CANSparkMax(Constants.BACKL, MotorType.kBrushless);
  public final CANSparkMax back_R = new CANSparkMax(Constants.BACKR, MotorType.kBrushless);

  private final SpeedControllerGroup left = new SpeedControllerGroup(front_L, back_L);  
  private final SpeedControllerGroup right = new SpeedControllerGroup(front_R, back_R); 
    
  private final DifferentialDrive drive = new DifferentialDrive(left, right);

  public final AHRS navx = new AHRS(I2C.Port.kMXP);

  private double P;
  private double I;
  private double D;
  private double SetPoint;

  public double kP = 0.0235; 
  public double kI = 0.00028;
  public double kD = 0.0023;
  public double set = 20;

  public CANEncoder leftEncoder = back_L.getEncoder();
  public CANEncoder rightEncoder = back_R.getEncoder();

  double output = 0;

  public DriveTrain() {
    drive.setSafetyEnabled(false);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("NavxHead", getHeading()); 
  }

  public void tankDrive(double leftSpeed, double rightSpeed){
    drive.tankDrive(-leftSpeed, -rightSpeed);
  }

  public void switchIdleMode(){
    if(front_L.getIdleMode() == IdleMode.kBrake){
      front_L.setIdleMode(IdleMode.kCoast);
      front_R.setIdleMode(IdleMode.kCoast);
      back_L.setIdleMode(IdleMode.kCoast);
      back_R.setIdleMode(IdleMode.kCoast);
    }else{
      front_L.setIdleMode(IdleMode.kBrake);
      front_R.setIdleMode(IdleMode.kBrake);
      back_L.setIdleMode(IdleMode.kBrake);
      back_R.setIdleMode(IdleMode.kBrake);
    }
  }

  public void arcadeDrive(double speed, double rot){
    drive.arcadeDrive(speed, rot);
  }

  public double getHeading(){
    return Math.IEEEremainder(navx.getAngle() , 360) * (Constants.kGyroReversed ? -1.0 : 1.0);
  }

  public String driveMode(){
    if(front_L.getIdleMode() == IdleMode.kBrake){
      return "Brake";
    }
    else{
      return "Coast";
    }

  }

  public void stop() {
    drive.tankDrive(0, 0);
  }

  public void rotate(double output) { 
    drive.arcadeDrive(0, output); // positive is right, negative is left
  }

  public double getLeftRotation(){
    return leftEncoder.getPosition();
  }

  public double getRightRotation(){
    return rightEncoder.getPosition();
  }
  
}
