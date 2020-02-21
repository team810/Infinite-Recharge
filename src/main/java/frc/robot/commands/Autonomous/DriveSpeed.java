/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveSpeed extends CommandBase {
  
  private DriveTrain d;
  private double startTime;
  private int time;

  public DriveSpeed(DriveTrain driveTrain, int time) {
    d = driveTrain;
    this.time = time;
    addRequirements(d);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(System.currentTimeMillis() - startTime < time){
      d.tankDrive(0.5, 0.5);
    }else{
      d.tankDrive(0, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    d.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean isFinished = System.currentTimeMillis() - startTime < time ? true : false;
    return isFinished;
  }
}