/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveBack extends CommandBase {
  
  DriveTrain d;
  double leftEnc, rightEnc;

  public DriveBack(DriveTrain d) {
    this.d = d;
    addRequirements(d);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    d.resetEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    d.arcadeDrive(-.5, 0);
    if(d.backLEncoder.getPosition() < -0.008304){
      d.arcadeDrive(0, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
