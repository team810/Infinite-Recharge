/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.RevShooter;
import frc.robot.commands.RunFeed;
import frc.robot.commands.RunShooter;
import frc.robot.commands.SetIdleMode;
import frc.robot.commands.SetSolenoid;
import frc.robot.commands.ToggleDriveMode;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class FarShooter extends SequentialCommandGroup {
  

  public FarShooter(Shooter s, Intake i, DriveTrain d) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(new SetIdleMode(d, true), 
          new SetSolenoid(s.shooterSOL, false),
          new RevShooter(s, 5100),
          new WaitCommand(.4), 
          new RunShooter(s, i, .85), 
          new SetIdleMode(d, false),
          new WaitCommand(.2));
  }
}
