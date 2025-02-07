package frc.robot;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.SwerveUtils.SwerveActions;
import frc.robot.SwerveUtils.SwerveWheel;

@Logged
public class Robot extends TimedRobot {
  private Joystick j1 = new Joystick(0);

  public SparkMax motorLeftFrontDirection = new SparkMax(Constants.MOTOR_LEFT_ANGULAR_FRONT, MotorType.kBrushless);
  public SparkMax motorLeftBackDirection = new SparkMax(Constants.MOTOR_LEFT_ANGULAR_BACK, MotorType.kBrushless);
  public SparkMax motorRightFrontDirection = new SparkMax(Constants.MOTOR_RIGHT_ANGULAR_FRONT, MotorType.kBrushless);
  public SparkMax motorRightBackDirection = new SparkMax(Constants.MOTOR_RIGHT_ANGULAR_BACK, MotorType.kBrushless);

  public SparkMax motorLeftFrontSpeed = new SparkMax(Constants.MOTOR_LEFT_DRIVER_FRONT, MotorType.kBrushless);
  public SparkMax motorLeftBackSpeed = new SparkMax(Constants.MOTOR_LEFT_DRIVER_BACK, MotorType.kBrushless);
  public SparkMax motorRightFrontSpeed = new SparkMax(Constants.MOTOR_RIGHT_DRIVER_FRONT, MotorType.kBrushless);
  public SparkMax motorRightBackSpeed = new SparkMax(Constants.MOTOR_RIGHT_DRIVER_BACK, MotorType.kBrushless);

  public CANcoder encoder1 = new CANcoder(Constants.CANCODER_FRONT_LEFT);
  public CANcoder encoder2 = new CANcoder(Constants.CANCODER_BACK_LEFT);
  public CANcoder encoder3 = new CANcoder(Constants.CANCODER_FRONT_RIGHT);
  public CANcoder encoder4 = new CANcoder(Constants.CANCODER_BACK_RIGHT);

  
  private final StructArrayPublisher<SwerveModuleState> publisher =
    NetworkTableInstance.getDefault()
      .getStructArrayTopic("/SwerveStates", SwerveModuleState.struct).publish();

  public Pigeon2 gyro = new Pigeon2(15);

  private final PIDController pids = new PIDController(Constants.KP_Swerve_ANGLE, Constants.KI_Swerve_ANGLE, Constants.KD_Swerve_ANGLE);

  SwerveWheel frontLeftSwerveWheel = new SwerveWheel(motorLeftFrontDirection, motorLeftFrontSpeed, encoder1, "1", pids);
  SwerveWheel backLeftSwerveWheel = new SwerveWheel(motorLeftBackDirection, motorLeftBackSpeed, encoder2, "2", pids);
  SwerveWheel frontRightSwerveWheel = new SwerveWheel(motorRightFrontDirection, motorRightFrontSpeed, encoder3, "3", pids);
  SwerveWheel backRightSwerveWheel = new SwerveWheel(motorRightBackDirection, motorRightBackSpeed, encoder4, "4", pids);

  SwerveModuleState frontLeftSwerveWheelState = new SwerveModuleState();
  SwerveModuleState backLeftSwerveWheelState = new SwerveModuleState();
  SwerveModuleState frontRightSwerveWheelState = new SwerveModuleState();
  SwerveModuleState backRightSwerveWheelState = new SwerveModuleState();

  SwerveWheel[] wheels = { frontLeftSwerveWheel,frontRightSwerveWheel, backLeftSwerveWheel, backRightSwerveWheel};
  SwerveWheel[] frontalWheels = {frontLeftSwerveWheel, frontRightSwerveWheel};
  SwerveWheel[] backWheels = {backLeftSwerveWheel, backRightSwerveWheel};
  private double finalpose;
  private double x1ToAngle, y1ToAngle, x2ToAngle, y2ToAngle, turnSpeed, l2, r2;
  private double magTranslade;
  private double yaw, roll, pitch, magneticZ;
  private double[] calibrationPID = {Constants.KP_Swerve_ANGLE, Constants.KI_Swerve_ANGLE, Constants.KD_Swerve_ANGLE};
  private boolean analog1Active, analog2Active;

  double angle1Translade;

  @Override
  public void robotInit() {
    pids.setTolerance(0.01);
    pids.setIZone(0.1);

  }

  @Override
  public void robotPeriodic() {

    yaw = gyro.getRotation2d().getDegrees();
    magneticZ = gyro.getMagneticFieldZ().getValueAsDouble();

    yaw = Units.degreesToRotations(yaw % 360);
    
    SmartDashboard.putNumber("z", (Math.IEEEremainder(yaw, 2))); 
    SmartDashboard.putNumber("z rad", Units.rotationsToRadians(yaw)); 

    frontLeftSwerveWheelState.speedMetersPerSecond = frontLeftSwerveWheel.getRotSpeedInSec();
    backLeftSwerveWheelState.speedMetersPerSecond = backLeftSwerveWheel.getRotSpeedInSec();
    frontRightSwerveWheelState.speedMetersPerSecond = frontRightSwerveWheel.getRotSpeedInSec();
    backRightSwerveWheelState.speedMetersPerSecond = backRightSwerveWheel.getRotSpeedInSec();

    frontLeftSwerveWheelState.angle = frontLeftSwerveWheel.getR2D().plus(new Rotation2d(yaw));
    backLeftSwerveWheelState.angle = backLeftSwerveWheel.getR2D().plus(new Rotation2d(yaw));
    frontRightSwerveWheelState.angle = frontRightSwerveWheel.getR2D().plus(new Rotation2d(yaw));
    backRightSwerveWheelState.angle = backRightSwerveWheel.getR2D().plus(new Rotation2d(yaw));

    publisher.set(new SwerveModuleState[] {
      frontLeftSwerveWheelState,
      frontRightSwerveWheelState,
      backLeftSwerveWheelState,
      backRightSwerveWheelState
    });

  }

  @Override
  public void teleopInit() {
    
    super.teleopInit();
  }

  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("Robot Heading", getRotation2d().getRotations());
    turnSpeed = j1.getRawAxis(Constants.RIGHT_ROT_AXIS);
    finalpose = j1.getPOV();

    if(j1.getRawButton(Constants.BNT_B))
      gyro.setYaw(0);

    x1ToAngle = -j1.getRawAxis(0);
    y1ToAngle = j1.getRawAxis(1);
    
    x2ToAngle = j1.getRawAxis(4);
    y2ToAngle = j1.getRawAxis(5);

    l2 = j1.getRawAxis(Constants.L2_TRIGGER);
    r2 = j1.getRawAxis(Constants.R2_TRIGGER);

    analog1Active = (Math.abs(x1ToAngle) >= Constants.kDeadband) || (Math.abs(y1ToAngle) >= Constants.kDeadband);
    analog2Active = (Math.abs(x2ToAngle) >= Constants.kDeadband) || (Math.abs(y2ToAngle) >= Constants.kDeadband);

    magTranslade = Math.sqrt(Math.pow(x1ToAngle, 2) + Math.pow(y1ToAngle, 2));

    angle1Translade = Units.radiansToRotations(Math.atan2(x1ToAngle, y1ToAngle) + Math.PI);
    
    angle1Translade += yaw;

    if (Math.abs(x1ToAngle) <= Constants.kDeadband && Math.abs(y1ToAngle) <= Constants.kDeadband)
      angle1Translade = -1;

    if (finalpose == 0)
      finalpose = 360;

    SmartDashboard.putNumber("finalpose", finalpose);
    SmartDashboard.putNumber("Analog1Translade", angle1Translade);

    if (analog1Active){
      SwerveActions.transladeTurn(frontalWheels, backWheels, angle1Translade, turnSpeed);
      SwerveActions.speedOn(wheels, (r2-l2));

      SmartDashboard.putString("TurnMode", "TransladeTurn");
    }
    else if (analog2Active){
      SwerveActions.turnIn(wheels, x2ToAngle);
      SwerveActions.speedOn(wheels, Math.copySign((r2-l2), x2ToAngle)); 
      SmartDashboard.putString("TurnMode", "TurnIn");
    }
    else {
      SwerveActions.motorsOff(wheels);
      SwerveActions.speedOn(wheels, (r2-l2));    
      SmartDashboard.putString("TurnMode", "Stopped");
    }

}

  @Override
  public void teleopExit() {
  }

  public double getRobotHeading() {
    return (Math.IEEEremainder(gyro.getYaw().getValueAsDouble(), 2));

  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getRobotHeading());


  }

  @Override
  public void autonomousInit() {
    
  }

  @Override
  public void autonomousPeriodic() {
    
  }

  @Override
  public void autonomousExit() {
  
  }
}
