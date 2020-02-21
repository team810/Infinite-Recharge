/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  
  public CANSparkMax intake = new CANSparkMax(Constants.INTAKE_MOTOR, MotorType.kBrushless);
  public CANSparkMax feeder = new CANSparkMax(Constants.FEED_MOTOR, MotorType.kBrushless);

  public DoubleSolenoid intakeSOL = new DoubleSolenoid(Constants.INTAKE_FORWARD, Constants.INTAKE_REVERSE);

  private AnalogInput ultrasonic = new AnalogInput(Constants.ULTRASONIC); 

  public Intake() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public DoubleSolenoid getIntakeSolenoid(){
    return intakeSOL;
  }

  public double getDistance(){
    return ultrasonic.getVoltage();
  }
}
