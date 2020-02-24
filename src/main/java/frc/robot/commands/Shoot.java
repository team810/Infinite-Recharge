/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;

public class Shoot extends CommandBase {
  
  Shooter s;
  Intake i;
  double shooterSpeed, feedSpeed, rpm;

  public Shoot(Shooter s, Intake i, double shooterSpeed, double feedSpeed, double rpm) {
    this.s = s;
    this.i = i;
    this.shooterSpeed = shooterSpeed;
    this.feedSpeed = feedSpeed;
    this.rpm = rpm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(s.shooter.getEncoder().getVelocity() < rpm){
      s.shooter.set(1);
      i.feeder.set(0);
    }else{
      s.shooter.set(shooterSpeed);
      i.feeder.set(feedSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s.shooter.set(0);
    i.feeder.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
