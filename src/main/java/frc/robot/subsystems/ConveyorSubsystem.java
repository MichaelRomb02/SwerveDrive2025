// FRC2106 Junkyard Dogs - Continuity Base Code - www.team2106.org

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants.ConveyorConstants;
import frc.robot.util.Constants.ElevatorConstants;
import frc.robot.util.Constants.ArmConstants;

import java.util.function.DoubleSupplier;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkClosedLoopController;

import com.revrobotics.spark.SparkBase.*;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.*;
import com.revrobotics.spark.config.*;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Ignore unused variable warnings
@SuppressWarnings("unused")

public class ConveyorSubsystem extends SubsystemBase{

    // Solenoid
    

    // Motors
    private WPI_TalonSRX ConveyorMotor;




    // Encoder and PIDf



    // Lift Subsystem Constructor
    public ConveyorSubsystem(){

        // Set solenoid object values
      
        

        // Set default state of soleno
        // Set motor object values take in CAN ID
        ConveyorMotor = new WPI_TalonSRX(ConveyorConstants.kConveyorMotorPort);

        // Make motor two follow motor one
       

        // Set motors to brake mode
        

    }

    @Override
    public void periodic(){
      
        
    }

    







    

    // Set both elevator motors to input
    public void ConveyorForward(){
        ConveyorMotor.set(ControlMode.PercentOutput,-1); 
    }
    public void ConveyorBackward(){
        ConveyorMotor.set(ControlMode.PercentOutput, 1); 
    }

    // Set both elevator motors to zero
    public void ConveyorStop(){
      ConveyorMotor.set(ControlMode.PercentOutput, 0);
    }



}
