/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  
  public CANSparkMax shooter = new CANSparkMax(Constants.SHOOT_MOTOR, MotorType.kBrushless);
  public DoubleSolenoid shooterSOL = new DoubleSolenoid(Constants.SHOOTER_FORWARD, Constants.SHOOTER_REVERSE);

  public Shooter() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getVelocity(){
    return shooter.getEncoder().getVelocity();
  }

  public String getShooterPosition(){
    if(shooterSOL.get() == Value.kForward){
      return "Close";
    }
    else{
      return "Far";
    }
  }
}
