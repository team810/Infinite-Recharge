/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.InstantCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class SetSolenoid extends InstantCommand {

  DoubleSolenoid dSolenoid;
  boolean value;

  public SetSolenoid(DoubleSolenoid dSolenoid, boolean value){
    this.dSolenoid = dSolenoid;
    this.value = value;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    dSolenoid.set((value) ? Value.kForward : Value.kReverse);
  }
}
