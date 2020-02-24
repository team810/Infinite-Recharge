/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Limelight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class SwitchStream extends InstantCommand {

  Limelight l;
  boolean isVision;

  public SwitchStream(Limelight l) {
    this.l = l;
    if(l.stream.getDouble(0) == 1){
      isVision = true;
    }
    if(l.stream.getDouble(0) == 2){
      isVision = false;
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    l.stream.setNumber(isVision ? 1 : 2);

  }
}
