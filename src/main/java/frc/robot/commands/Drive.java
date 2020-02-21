/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;

public class Drive extends CommandBase {
  /**
   * Creates a new Drive.
   */
  DoubleSupplier leftSpeed, rightSpeed;
  DriveTrain d;
  Limelight l;

  public Drive(Limelight l ,DriveTrain d, DoubleSupplier leftSpeed, DoubleSupplier rightSpeed) {
    this.d = d;
    this.l = l;
    this.leftSpeed = leftSpeed;
    this.rightSpeed = rightSpeed;
    addRequirements(d);
    addRequirements(l);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    l.lightOff();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    d.tankDrive(leftSpeed.getAsDouble(), rightSpeed.getAsDouble());
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
