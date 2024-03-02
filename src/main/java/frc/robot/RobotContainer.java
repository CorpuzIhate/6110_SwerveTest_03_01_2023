// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.IntakeCMD;
import frc.robot.commands.LockOnCMD;
import frc.robot.commands.MoveIntakeArmCMD;
import frc.robot.commands.ShooterCMD;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeOutakeSub;
import frc.robot.subsystems.LimeLightSub;
import frc.robot.subsystems.ShooterSub;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.SwerveSub;

import java.nio.file.Path;

import org.ietf.jgss.Oid;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OIConstants.kDriverControllerPort);

  private final SwerveSub swerveSub =  new SwerveSub();
  private final LimeLightSub limeLightSub = new LimeLightSub();
  private final IntakeOutakeSub intakeOutakeSub = new IntakeOutakeSub();
  private final ShooterSub shooterSub = new ShooterSub();

  private final LockOnCMD lockOnCMD = new LockOnCMD(swerveSub, limeLightSub);

  private final Joystick driverJoyStick = new Joystick(OIConstants.kDriverControllerPort);

  private final JoystickButton FieldOrientButton = new JoystickButton(driverJoyStick, OIConstants.kDriverFieldOrientedButtonIdx);
  private final JoystickButton LimelightOrientButton = new JoystickButton(driverJoyStick, OIConstants.kDriveLimeOrientButtonIdx);
  private boolean fieldOrientToggle;
  //autos
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    fieldOrientToggle = driverJoyStick.getRawButtonPressed(OIConstants.kDriverFieldOrientedButtonIdx);
    

    // Configure the trigger bindings
    swerveSub.setDefaultCommand(new SwerveJoystickCmd(
      swerveSub, limeLightSub,
      () -> -driverJoyStick.getRawAxis(OIConstants.kDriverYAxis),
      () -> driverJoyStick.getRawAxis(OIConstants.kDriverXAxis),
      () -> driverJoyStick.getRawAxis(OIConstants.kDriverRotAxis),
      fieldOrientToggle,
      () -> !driverJoyStick.getRawButton(OIConstants.kDriveLimeOrientButtonIdx))); // by defualt will work on fields reference frame
    
    intakeOutakeSub.setDefaultCommand(new IntakeCMD(intakeOutakeSub, false, false)); //needs outake and intake bool -> both false to default the motors to 0
    


    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
   
   new JoystickButton(driverJoyStick, OIConstants.kDriveIntakeButtonIdx).whileTrue(new IntakeCMD(intakeOutakeSub, true, false));
   new JoystickButton(driverJoyStick, OIConstants.kDriveOutakeButtonIdx).whileTrue(new IntakeCMD(intakeOutakeSub, false, true));
   new JoystickButton(driverJoyStick, OIConstants.kDriveShooterButtonIdx).whileTrue(new ShooterCMD(shooterSub,  true));

      //move arm to position

   // currentIntakeArmPosSetpoint
   new JoystickButton(driverJoyStick, OIConstants.kMoveIntakeArmToUpPosButtonIdx)
   .onTrue(new MoveIntakeArmCMD(intakeOutakeSub, IntakeConstants.kIntakeArmUpPosSetpoint)); // button 7
  
   new JoystickButton(driverJoyStick, OIConstants.kMoveIntakeArmToDownPosButtonIdx)
   .onTrue(new MoveIntakeArmCMD(intakeOutakeSub, IntakeConstants.kIntakeArmDownPosSetpoint));

   new JoystickButton(driverJoyStick, OIConstants.kMoveIntakeArmToMidPosButtonIdx)
   .onTrue(new MoveIntakeArmCMD(intakeOutakeSub,IntakeConstants.kIntakeArmMidPosSetpoint)); // button 9
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous


    //PathPlannerPath path =  PathPlannerPath.fromPathFile("test"); // wil run the auto named
    // test in the pathPlanner GUI

    //return AutoBuilder.followPath(path);



    return new PathPlannerAuto("test_2024_02_29");


  // return new PathPlannerAuto("test");
  }
}