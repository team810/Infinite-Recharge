/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.CamMode;
import frc.robot.commands.CamVisionMode;
import frc.robot.commands.Drive;
import frc.robot.commands.RunFeed;
import frc.robot.commands.RunIntake;
import frc.robot.commands.ToggleDriveMode;
import frc.robot.commands.ToggleSolenoid;
import frc.robot.commands.VisionMode;
import frc.robot.commands.Autonomous.CloseShooter;
import frc.robot.commands.Autonomous.DriveSpeed;
import frc.robot.commands.Autonomous.FarShooter;
import frc.robot.commands.Autonomous.TurnToTarget;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ControlPanel;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  
  public DriveTrain m_drive = new DriveTrain();
  Shooter m_shoot = new Shooter();
  ControlPanel m_cp = new ControlPanel();
  Intake m_intake = new Intake();
  Climber m_climb = new Climber();
  Limelight m_limelight = new Limelight();

  private final Joystick gamepad = new Joystick(4);
  public final XboxController gp = new XboxController(5);
  public static final Joystick left = new Joystick(Constants.LEFT_STICK);
  public static final Joystick right = new Joystick(Constants.RIGHT_STICK);

  private final Command m_autoCommand = new TurnToTarget(m_drive, m_drive.set);//Robot.getAngleToTarget()

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    m_drive.setDefaultCommand(new Drive(m_limelight ,m_drive, 
                                                ()-> left.getRawAxis(1), () -> right.getRawAxis(1)));
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
/** */
    final JoystickButton intake = new JoystickButton(gamepad, Constants.INTAKE_BTN);
    intake.whileHeld(new RunIntake(m_intake, -.6));

    final JoystickButton intakePrimary = new JoystickButton(right, Constants.INTAKE_BTN);
    intakePrimary.whileHeld(new RunIntake(m_intake, .6));

    final JoystickButton feeder = new JoystickButton(gamepad, Constants.FEEDER_BTN);
    feeder.whileHeld(new RunFeed(m_intake, .4));

    final JoystickButton feederReverse = new JoystickButton(gamepad, Constants.FEED_REVERSE_BTN);
    feederReverse.whileHeld(new RunFeed(m_intake, -.4));

    final JoystickButton farShoot = new JoystickButton(gamepad, Constants.FAR_SHOOT_BTN);
    farShoot.whileHeld(new FarShooter(m_shoot, m_intake, m_drive));

    final JoystickButton intakeSOL = new JoystickButton(gamepad, Constants.INTAKE_SOL_BTN);
    intakeSOL.whenPressed(new ToggleSolenoid(m_intake.intakeSOL));

    final JoystickButton cpSOL = new JoystickButton(gamepad, 11);
    cpSOL.whenPressed(new ToggleSolenoid(m_cp.control_solenoid));

    final JoystickButton closeShoot = new JoystickButton(gamepad, Constants.CLOSE_SHOOT_BTN);
    closeShoot.whileHeld(new CloseShooter(m_shoot, m_intake, m_drive));

    final JoystickButton armToggle = new JoystickButton(gamepad, Constants.ARMSOL_BTN);
    armToggle.whenPressed(new ToggleSolenoid(m_climb.switch_climb));

    final JoystickButton switchDriveMode = new JoystickButton(gamepad, Constants.DRIVE_MODE_BTN);
    switchDriveMode.whenPressed(new ToggleDriveMode(m_drive));

    final JoystickButton switchShooter = new JoystickButton(gamepad, Constants.SWITCH_SHOOT);
    switchShooter.whenPressed(new ToggleSolenoid(m_shoot.shooterSOL));

    final JoystickButton visionMode = new JoystickButton(right, 3);
    visionMode.toggleWhenActive(new VisionMode(m_limelight));
    final JoystickButton camMode = new JoystickButton(right, 4);
    camMode.toggleWhenActive(new CamMode(m_limelight));
    final JoystickButton camVisionMode = new JoystickButton(right, 2);
    camVisionMode.toggleWhenActive(new CamVisionMode(m_limelight));

    final JoystickButton driveBack = new JoystickButton(left, 1);
    driveBack.whenPressed(new DriveSpeed(m_drive, 2000));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoCommand;
  }
}
