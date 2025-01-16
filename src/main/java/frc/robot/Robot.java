package frc.robot;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private Joystick j1 = new Joystick(0);
  
  public SparkMax motorLeftFrontDirection = new SparkMax(Constants.MOTOR_LEFT_ANGULAR_FRONT, MotorType.kBrushless);
  public SparkMax motorLeftBackDirection = new SparkMax(Constants.MOTOR_LEFT_ANGULAR_BACK, MotorType.kBrushless);
  public SparkMax motorRightFrontDirection = new SparkMax(Constants.MOTOR_RIGHT_ANGULAR_FRONT, MotorType.kBrushless);
  public SparkMax motorRightBackDirection = new SparkMax(Constants.MOTOR_RIGHT_ANGULAR_BACK, MotorType.kBrushless);

  public SparkMax motorLeftFrontSpeed = new SparkMax(Constants.MOTOR_LEFT_DRIVER_FRONT, MotorType.kBrushless);
  public SparkMax motorLeftBackSpeed = new SparkMax(Constants.MOTOR_LEFT_DRIVER_BACK, MotorType.kBrushless);
  public SparkMax motorRightFrontSpeed = new SparkMax(Constants.MOTOR_RIGHT_ANGULAR_FRONT, MotorType.kBrushless);
  public SparkMax motorRightBackSpeed = new SparkMax(Constants.MOTOR_RIGHT_DRIVER_BACK, MotorType.kBrushless);

  public CANcoder encoder1 = new CANcoder(Constants.CANCODER_FRONT_LEFT);
  public CANcoder encoder2 = new CANcoder(Constants.CANCODER_BACK_LEFT);
  public CANcoder encoder3 = new CANcoder(Constants.CANCODER_FRONT_RIGHT);
  public CANcoder encoder4 = new CANcoder(Constants.CANCODER_BACK_RIGHT); 
 
  private double finalpose;
  private double actualpose1, actualpose2, actualpose3, actualpose4;

  @Override
  public void robotInit() {

  }

  @Override
  public void teleopPeriodic() {
    
   actualpose1 = getCANcoderDegrees(encoder1.getPosition().getValueAsDouble());
   actualpose2 = getCANcoderDegrees(encoder2.getPosition().getValueAsDouble());
   actualpose3 = getCANcoderDegrees(encoder3.getPosition().getValueAsDouble());
   actualpose4 = getCANcoderDegrees(encoder4.getPosition().getValueAsDouble());
    

    finalpose = j1.getPOV();
    SmartDashboard.putNumber("finalpose", finalpose);

     pov();

  }

  public double getCANcoderDegrees(double pose){

    double degrees = ((pose +1) %1 ) *360;

    if(degrees < 0) {
        degrees +=360;  
    }
    return degrees;
  }

  public void turnIn(){

  }

  public void setAllDir(double dirFL, double dirBL, double dirFR, double dirBR){

  }


  public void pov(){

    double tolerance = 10.0;
    SmartDashboard.putNumber("actualpose", actualpose1);

    if(finalpose != -1){
    if(Math.abs(actualpose1 - finalpose) > tolerance){

      motorLeftFrontDirection.set(0.05);
    
   }
   else if (Math.abs(actualpose1 - finalpose) < tolerance) {

       motorLeftFrontDirection.stopMotor();
       
   }
 
  }
  else{
    motorLeftFrontDirection.stopMotor();
  }
  }

}