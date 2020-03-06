/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;

public class track extends CommandBase {
  Limelight l;
  DriveTrain d;
  double kp = 0.1;
  double output = 0.1;
  double adjust;
  double tx = Constants.tx.getDouble(0.0);
  double left, right = 0;
  public track(Limelight l, DriveTrain d) {
    this.l = l;
    this.d = d;
    addRequirements(l, d);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double heading_error = -tx;
    if (tx > 0.0)
    {
      d.arcadeDrive(0, .4);
    }
    else if (tx < 0.0)
    {
      d.arcadeDrive(0, -.4);
    }
    SmartDashboard.putNumber("tX", Constants.tx.getDouble(0.0));
    tx = Constants.tx.getDouble(0.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    d.arcadeDrive(0, 0);
  }



  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
