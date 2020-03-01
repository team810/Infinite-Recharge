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
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  public final CANSparkMax front_L = new CANSparkMax(Constants.FRONTL, MotorType.kBrushless);
  public final CANSparkMax front_R = new CANSparkMax(Constants.FRONTR, MotorType.kBrushless);
  public final CANSparkMax back_L =  new CANSparkMax(Constants.BACKL, MotorType.kBrushless);
  public final CANSparkMax back_R = new CANSparkMax(Constants.BACKR, MotorType.kBrushless);

    
  private final DifferentialDrive drive = new DifferentialDrive(back_L, front_R);

  public final AHRS navx = new AHRS(I2C.Port.kMXP);

  public double kP = 0.0235; 
  public double kI = 0.00028;
  public double kD = 0.0023;
  public double set = 20;

  public double targetDeg = 20;

  public CANEncoder backLEncoder = back_L.getEncoder();
  public CANEncoder backREncoder = back_R.getEncoder();
  public CANEncoder frontLEncoder = front_L.getEncoder();
  public CANEncoder frontREncoder = front_R.getEncoder();

  double output = 0;

  double encoderConstant = (1 / 8.68) * 0.155 * Math.PI; 

  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(28));
  //DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading());

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.268, 1.89, 0.243);

  PIDController leftPidController = new PIDController(9.95, 0, 0);
  PIDController rightPidController = new PIDController(9.95, 0, 0);

  Pose2d pose;

  public static final double kDistancePerRevolution = 18.84;  // guestimate from your code
  public static final double kPulsesPerRevolution = 1024;     // for an AS5145B Magnetic Encoder
  public static final double kDistancePerPulse = 1./256.;

  public DriveTrain() {
    drive.setSafetyEnabled(false);
    back_L.restoreFactoryDefaults();
    back_R.restoreFactoryDefaults();
    front_R.restoreFactoryDefaults();
    front_L.restoreFactoryDefaults();
    
    backLEncoder.setPositionConversionFactor(encoderConstant);
    frontREncoder.setPositionConversionFactor(encoderConstant);
    backLEncoder.setPositionConversionFactor(encoderConstant / 60);
    frontREncoder.setPositionConversionFactor(encoderConstant / 60);

    front_L.follow(back_L);
    back_R.follow(front_R);

  }

  @Override
  public void periodic() {
    //pose = odometry.update(getHeading(), getSpeeds(true), getSpeeds(false));
    SmartDashboard.putNumber("Navx Heading", getNavxAngle());
  }

  public void tankDrive(double leftSpeed, double rightSpeed){
    drive.tankDrive(-leftSpeed, -rightSpeed, true);
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

  //public Rotation2d getHeading(){
  //  return Rotation2d.fromDegrees(-navx.getAngle());
  //}

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

  public double getLeftDistance(){
    return backLEncoder.getPosition();
  }

  public double getRightDistance(){
    return frontREncoder.getPosition();
  }

  public double getLeftVelocity(){
    return backLEncoder.getVelocity();
  }

  public double getRightVelocity(){
    return backLEncoder.getVelocity();
  }

  public void resetEncoder(){
    backLEncoder.setPosition(0);
    frontREncoder.setPosition(0);
  }

  public double getSpeeds(final boolean isLeft) {
    if(isLeft){
      return front_L.getEncoder().getVelocity() / Constants.GEAR_RATIO * 2 * Math.PI * Units.inchesToMeters(Constants.WHEEL_DIAMETER);
    }else{
      return front_R.getEncoder().getVelocity() / Constants.GEAR_RATIO * 2 * Math.PI * Units.inchesToMeters(Constants.WHEEL_DIAMETER);
    }
  }

  public DifferentialDriveWheelSpeeds getSpeeds() {
    return new DifferentialDriveWheelSpeeds(front_L.getEncoder().getVelocity() / Constants.GEAR_RATIO * 2 * Math.PI * Units.inchesToMeters(Constants.WHEEL_DIAMETER),
    front_R.getEncoder().getVelocity() / Constants.GEAR_RATIO * 2 * Math.PI * Units.inchesToMeters(Constants.WHEEL_DIAMETER));
  }

  public PIDController getLeftPIDController(){
    return leftPidController;
  }

  public double getHeading(){
    return Math.IEEEremainder(navx.getAngle(), 360) * (Constants.kGyroReversed ? -1.0 : 1.0);
  }

  public PIDController getRightPIDController(){
    return rightPidController;
  }

  public DifferentialDriveKinematics getKinematics(){
    return kinematics;
  }

  public Pose2d getPose(){
    return pose;
  }

  public SimpleMotorFeedforward getFeedForward(){
    return feedforward;
  }

  public void setOutput(double leftVolts, double rightVolts){
    SmartDashboard.putNumber("Left Voltage", leftVolts);
    SmartDashboard.putNumber("Right Voltage", rightVolts);

    front_L.set(leftVolts/12);
    front_R.set(rightVolts/12);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    front_L.setVoltage(leftVolts);
    front_R.setVoltage(-rightVolts);
    drive.feed();
  }

  public void zeroHeading() {
    navx.reset();
  }

  public double getNavxAngle(){
    return navx.getAngle();
  }
}
