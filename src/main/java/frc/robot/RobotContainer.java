// FRC2106 Junkyard Dogs - Continuity Base Code - www.team2106.org

package frc.robot;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.commands.Arm.ArmIn;
import frc.robot.commands.Arm.ArmOut;
import frc.robot.commands.Arm.ArmMoveU;
import frc.robot.commands.Arm.ArmMoveD;
import frc.robot.commands.Arm.ArmStop;
import frc.robot.commands.Arm.ArmRotStop;
import frc.robot.commands.Conveyor.ConveyorForward;
import frc.robot.commands.Conveyor.ConveyorBackward;
import frc.robot.commands.Conveyor.ConveyorStop;
import frc.robot.commands.Elevator.ElevatorD;
import frc.robot.commands.Elevator.ElevatorU;
import frc.robot.commands.Elevator.ElevatorStop;
import frc.robot.commands.swerve.SwerveJoystick;

import frc.robot.commands.swerve.SwerveReset;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.util.Constants;
import frc.robot.util.Transmitter;
import frc.robot.util.Constants.AutoConstants;
import frc.robot.util.Constants.IOConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

// Ignore unused variable warnings
@SuppressWarnings("unused")

public class RobotContainer {

  // Converted to 2023 wpiblib

  //------------------------------------O-B-J-E-C-T-S-----------------------------------//

  // Deprecated joysticks
  //private final Joystick DriveJoystick = new Joystick(0);
  private final Joystick tx12 = new Joystick(0);
  private final Joystick ShooterJoystick = new Joystick(1);

  // Create tx16s transmitter

 private final CommandJoystick tx12COMD = new CommandJoystick(0);

  // Create led strips
 // private final LightStrip strips = new LightStrip(tx16s,0);



  // Create ultrasonic sensor 
  //private final UltrasonicRangefinder ultrasonic = new UltrasonicRangefinder(strips);

  // Create swerve subsystem
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  
  // Create vision subsystem


  // Create grabber subsystem

  // Create elevator subsystem
  
  

  private final ArmSubsystem ArmSubsystem = new ArmSubsystem();
  
  private final ConveyorSubsystem ConveyorSubsystem = new ConveyorSubsystem();

  private final ElevatorSubsystem ElevatorSubsystem = new ElevatorSubsystem();

  //initiate the sendable chooser - 7038, ree
  SendableChooser <Command> m_chooser = new SendableChooser<>(); //when you want to do paths you should do SendableChooser<Auto> instead - 7038, ree
  //
  //initiate the camera

  // Create PID controllers for trajectory tracking

  //public final ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, AutoConstants.kIThetaController, AutoConstants.kDThetaController, AutoConstants.kThetaControllerConstraints);
  

  // Create xbox controller
  //private final XboxController xbox = new XboxController(3);

  //private final VL53L4CX vl53l4cx = new VL53L4CX(20000);

  //----------------------A-U-T-O---C-O-M-M-A-N-D-S----------------------------//

  // Command chooser for auto




  //Command REDtwoPieceWithLessPaths = new TwoPieceWithLessPaths(swerveSubsystem, elevatorSubsystem, grabberSubsystem, xController, yController, ppThetaController, strips);
  //------------------------------------C-O-N-S-T-R-U-C-T-O-R----------------------------//

  public RobotContainer(){

    //CameraServer.startAutomaticCapture();
    //PathPlannerServer.startServer(5811);


    
    
    SmartDashboard.putData(m_chooser);
    
    //adding in camera to smartdashboard - 7038, ree



    // Set swerve subsystem default command to swerve joystick with respective joystick inputs
  //>--------------O-L-D--T-R-A-N-S----------------//

  // Joystick Numbers 0 = LEFT : 1 = RIGHT
  // Joystick Axises: 0 = left/right : 1 = forward/backwards : 2 = dial
  // OLD-> Transmitter Axises: 0 = roll : 1 = pitch : 2 = throttle : 3 = yaw : 4 = analog1 : 5 = analog2

  
    swerveSubsystem.setDefaultCommand(new SwerveJoystick(swerveSubsystem,
    () -> tx12.getRawAxis(1), // X-Axis
    () -> -tx12.getRawAxis(3), // Y-Axis
    () -> tx12.getRawAxis(0), // R-Axis
    () -> tx12.getRawButton(0), // Field oriented -does nothing right now
    () -> swerveSubsystem.getHeading(), // Navx heading
    () -> tx12.getRawButton(0))); // Flick offset button, should be toggle!

 //>-----------T-X-1-6-S---------<//

/*swerveSubsystem.setDefaultCommand(new SwerveJoystick(swerveSubsystem,
() -> -DriveJoystick.getRawAxis(0), // X-Axis
() -> DriveJoystick.getRawAxis(1), // Y-Axis
() -> -DriveJoystick.getRawAxis(2), // R-Axis
() -> DriveJoystick.getRawButton(2), // Field oriented -does nothing right now
() -> swerveSubsystem.getHeading(), // Navx heading
() -> DriveJoystick.getRawButton(4))); // Flick offset button, should be toggle!

 //AngleSubsystem.setDefaultCommand(new AngleMove(AngleSubsystem, ShooterJoystick.getRawAxis(1)));
  //>----------S-E-N-D-E-R----------<//

    // Run button binding method
    configureButtonBindings();

  //>---------D-E-F-A-U-L-T----------<//

  /* elevatorSubsystem.setDefaultCommand(new ElevatorJoystick(elevatorSubsystem,
  () -> xbox.getRawAxis(1))); */

  /* grabberSubsystem.setDefaultCommand(new GrabberTrigger(grabberSubsystem,
  () -> xbox.getRawAxis(3))); */

  }

  //------------------------------------D-E-B-U-G------------------------------------//

  private double zeroFunct(){return 0;}

  private boolean trueFunct(){return true;}

  private boolean falseFunct(){return false;}

  //------------------------------------B-U-T-T-O-N-S------------------------------------//

  // Create button bindings
  private void configureButtonBindings(){

    // ELEVATOR SOLENOID
   

    // 4 Y - Garbber Solenoid
    //xbox.y().onTrue(new GrabberSolenoid(grabberSubsystem));

    // 1 A - Grabber Intake Forward
    //xbox.a().onTrue(new GrabberForward(grabberSubsystem));
    //xbox.a().toggleOnFalse(new GrabberHold(grabberSubsystem));

    // 2 B - Grabber Intake Reverse
    //xbox.b().onTrue(new GrabberReverse(grabberSubsystem));
    //xbox.b().toggleOnFalse(new GrabberHold(grabberSubsystem));

    // 3 X - Elevator Zero
    //xbox.x().onTrue(new ElevatorZero(elevatorSubsystem, grabberSubsystem));

    // 5 LB - Low Score
    //xbox.leftBumper().onTrue(new ScoreBottom(elevatorSubsystem, grabberSubsystem));

    // 6 RB - High Score
    //xbox.rightBumper().onTrue(new ScoreTop(elevatorSubsystem, grabberSubsystem));

    // 9 LJ - Loading station
   // xbox.button(9).onTrue(new LoadPlatform(elevatorSubsystem, grabberSubsystem));

    // 10 RJ - GrabrveMove(swerveSubsystem,
   // () -> swerveSubsystem.getHber angle zero
    //xbox.button(10).onTrue(new Sweeading(), 1.0,1.0));

   //xbox.button(10).onTrue(new AutoBalance(swerveSubsystem, strips));
   //xbox.button(10).onTrue(new LoadSlide(elevatorSubsystem, grabberSubsystem));

    // 10 RJ - Reset Odometry
   // xbox.button(10).onTrue(new ResetOdometry(swerveSubsystem, new Pose2d()));
   // xbox.button(10).onTrue(new ResetOdometry(swerveSubsystem, new Pose2d()));

    // Manual grabber angle test code
   // xbox.axisGreaterThan(1, 0.55).onTrue(new GrabberTrigger(grabberSubsystem, () -> xbox.getRawAxis(1)));
    
    // D-PAD LED Color selection
   // xbox.povUp().toggleOnTrue(new SetLedGameObject(leds, true));

   // xbox.povDown().toggleOnTrue(new SetLedGameObject(leds, false));

    //xbox.povLeft().toggleOnTrue(new SetLedWhiteMode(leds, "strobe"));
    //xbox.povRight().toggleOnTrue(new SetLedRed(strips));

    //tx16sCOMD.axisGreaterThan(1, 50.0).toggleOnTrue(new ElevatorZero(elevatorSubsystem, grabberSubsystem));

    // Homing
    new JoystickButton(ShooterJoystick, 8).whileTrue(new ArmIn(ArmSubsystem));
      new JoystickButton(ShooterJoystick, 8).onFalse(new ArmStop(ArmSubsystem));
    
    new JoystickButton(ShooterJoystick, 7).whileTrue(new ArmOut(ArmSubsystem));
      new JoystickButton(ShooterJoystick, 7).onFalse(new ArmStop(ArmSubsystem));
    
    new JoystickButton(ShooterJoystick, 4).whileTrue(new ArmMoveD(ArmSubsystem));
      new JoystickButton(ShooterJoystick, 4).onFalse(new ArmRotStop(ArmSubsystem));
    
    new JoystickButton(ShooterJoystick, 6).whileTrue(new ArmMoveU(ArmSubsystem));
      new JoystickButton(ShooterJoystick, 6 ).onFalse(new ArmRotStop(ArmSubsystem));
  



    new JoystickButton(ShooterJoystick, 1).whileTrue(new ConveyorForward(ConveyorSubsystem));
    new JoystickButton(ShooterJoystick, 1).onFalse(new ConveyorStop(ConveyorSubsystem));
    
    new JoystickButton(ShooterJoystick, 2).whileTrue(new ConveyorBackward(ConveyorSubsystem));
    new JoystickButton(ShooterJoystick, 2).onFalse(new ConveyorStop(ConveyorSubsystem));
   
    new JoystickButton(ShooterJoystick, 3).whileTrue(new ElevatorD(ElevatorSubsystem));
    new JoystickButton(ShooterJoystick, 3).onFalse(new ElevatorStop(ElevatorSubsystem));
   
    new JoystickButton(ShooterJoystick, 5).whileTrue(new ElevatorU(ElevatorSubsystem));
    new JoystickButton(ShooterJoystick, 5).onFalse(new ElevatorStop(ElevatorSubsystem));

    // Apriltag

    // new JoystickButton(xbox, 2).onTrue(new ElevatorApriltag(elevatorSubsystem, visionSubsystem));
    // Meters
    // new JoystickButton(xbox, 3).onTrue(new ElevatorMeters(elevatorSubsystem, 1.0));

    //--------------// Auto Bindings

    // Auto align with side station
   // new JoystickButton(tx16s, 8).onTrue(new LoadSlideAlign(swerveSubsystem, elevatorSubsystem, grabberSubsystem, leds, () -> -tx16sCOMD.getRawAxis(0), () -> tx16sCOMD.getRawAxis(1), () -> tx16s.getRawButton(8)));

    // Apriltag
    //new JoystickButton(tx16s, 8).onTrue(new SwerveAlignBasic(swerveSubsystem, visionSubsystem,
    //   () -> swerveSubsystem.getHeading(), () -> tx16s.getRawButton(8), () -> tx16s.getRawAxis(5)));
    
    // Run autonmous command during teleop
    //new JoystickButton(tx16s, 3).onTrue(new TrajectoryWeaver(swerveSubsystem,xController,yController,ppThetaController, pathOne, true));
    //new JoystickButton(tx16s, 7).onTrue(new ElevatorManual(elevatorSubsystem, 0.5));
    //new JoystickButton(tx16s, 7).onFalse(new ElevatorStop(elevatorSubsystem));
  }

  //------------------------------------R-E-F-E-R-R-E-R-S------------------------------------//

    //public void containerResetAllEncoders(){ swerveSubsystem.resetAllEncoders();}

    public Command getAutonomousCommand() {
        // TODO Auto-generated method stub
       // throw new UnsupportedOperationException("Unimplemented method 'getAutonomousCommand'");

     //  return shootCommand();
      Command autoCommand =  m_chooser.getSelected();

      return autoCommand;
       //return m_chooser - should return the chosen command from the smartdashboard
    }

    /*  public Command shootCommand() {
      Command shootSpeaker = new SequentialCommandGroup(
      new InstantCommand(ShooterSubsystem::toggleShooterIntake),
      new WaitCommand(.5),
      new InstantCommand(ShooterSubsystem::stopAll),
      new InstantCommand(ShooterSubsystem::toggleShooterSpeaker),
      //new InstantCommand(ShooterSubsystem::toggleShooterSpeakerright),
      //new InstantCommand(ShooterSubsystem::toggleShooterSpeakerleft),
      new InstantCommand(AngleSubsystem::AngleMove), 
      new WaitCommand(1.125), 
      new InstantCommand(AngleSubsystem::AngleStop),
      new InstantCommand(ShooterSubsystem::ShootNote),
      new WaitCommand(1.5),
      new InstantCommand(ShooterSubsystem::stopShooter),
      new InstantCommand(ShooterSubsystem::stopHold)
      );  

      return shootSpeaker;
    }
    public Command shootCommandright() {
      Command shootSpeaker = new SequentialCommandGroup(
      new InstantCommand(ShooterSubsystem::toggleShooterIntake),
      new WaitCommand(.5),
      new InstantCommand(ShooterSubsystem::stopAll),
      //new InstantCommand(ShooterSubsystem::toggleShooterSpeaker),
      new InstantCommand(ShooterSubsystem::toggleShooterSpeakerright),
      //new InstantCommand(ShooterSubsystem::toggleShooterSpeakerleft),
      new InstantCommand(AngleSubsystem::AngleMove), 
      new WaitCommand(1.125), 
      new InstantCommand(AngleSubsystem::AngleStop),
      new InstantCommand(ShooterSubsystem::ShootNote),
      new WaitCommand(1.5),
      new InstantCommand(ShooterSubsystem::stopShooter),
      new InstantCommand(ShooterSubsystem::stopHold)
      );

      return shootSpeaker;
    }
    public Command shootCommandleft() {
      Command shootSpeaker = new SequentialCommandGroup(
      new InstantCommand(ShooterSubsystem::toggleShooterIntake),
      new WaitCommand(.5),
      new InstantCommand(ShooterSubsystem::stopAll),
      //new InstantCommand(ShooterSubsystem::toggleShooterSpeaker),
      //new InstantCommand(ShooterSubsystem::toggleShooterSpeakerright),
      new InstantCommand(ShooterSubsystem::toggleShooterSpeakerleft),
      new InstantCommand(AngleSubsystem::AngleMove), 
      new WaitCommand(1.125), 
      new InstantCommand(AngleSubsystem::AngleStop),
      new InstantCommand(ShooterSubsystem::ShootNote),
      new WaitCommand(1.5),
      new InstantCommand(ShooterSubsystem::stopShooter),
      new InstantCommand(ShooterSubsystem::stopHold)
      );

      return shootSpeaker;
    } */
  //------------------------------------A-U-T-O-N-O-M-O-U-S------------------------------------//
  
  // Return the command to run during auto
  
}