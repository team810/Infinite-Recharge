/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ControlPanel extends SubsystemBase {
  
  public final CANSparkMax control_motor;
  public final DoubleSolenoid control_solenoid;


  public ControlPanel() {

    control_motor = new CANSparkMax(Constants.CONTROL_PANEL_MOTOR, MotorType.kBrushless);
    control_motor.setIdleMode(IdleMode.kBrake);
    control_solenoid = new DoubleSolenoid(Constants.CONTROL_EXTEND, Constants.CONTROL_RETRACT);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void stop(){
    control_motor.set(0);
  }

  public String getCPPosition(){
    if(control_solenoid.get() == Value.kForward){
      return "Up";
    }
    else{
      return "Down";
    }
  }

  public void init(){
    control_solenoid.set(Value.kReverse);
  }
}
