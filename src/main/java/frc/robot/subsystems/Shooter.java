/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.io.ByteArrayOutputStream;

import javax.sound.sampled.AudioFormat;
import javax.sound.sampled.AudioSystem;
import javax.sound.sampled.DataLine;
import javax.sound.sampled.TargetDataLine;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  
  public CANSparkMax shooter = new CANSparkMax(Constants.SHOOT_MOTOR, MotorType.kBrushless);
  public DoubleSolenoid shooterSOL = new DoubleSolenoid(Constants.SHOOTER_FORWARD, Constants.SHOOTER_REVERSE);

  ByteArrayOutputStream byteArrayOutputStream;
  TargetDataLine targetDataLine;
  int cnt;
  boolean stopCapture = false;
  byte tempBuffer[] = new byte[8000];
  int countzero, countdownTimer;    
  short convert[] = new short[tempBuffer.length];
 

  public Shooter() {

  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Valid", true);

  }

  public double getSound(){
    try {
      byteArrayOutputStream = new ByteArrayOutputStream();
      stopCapture = false;
      countdownTimer = 0;
      while (!stopCapture) {
          AudioFormat audioFormat = new AudioFormat(8000.0F, 16, 1, true, false);
          DataLine.Info dataLineInfo = new DataLine.Info(TargetDataLine.class, audioFormat);
          targetDataLine = (TargetDataLine) AudioSystem.getLine(dataLineInfo);
          targetDataLine.open(audioFormat);
          targetDataLine.start();
          cnt = targetDataLine.read(tempBuffer, 0, tempBuffer.length);
          byteArrayOutputStream.write(tempBuffer, 0, cnt); 
          try {
              countzero = 0;
              for (int i = 0; i < tempBuffer.length; i++) {                                     
                  convert[i] = tempBuffer[i];
                  if (convert[i] == 0) {
                      return countzero++;
                  }
              }
               
              countdownTimer++;
              System.out.println(countzero);

          } catch (StringIndexOutOfBoundsException e) {
              System.out.println(e.getMessage());
          }
          Thread.sleep(0);
          targetDataLine.close();
      }
  } catch (Exception e) {
      System.out.println(e);
  }

  return countzero;

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
