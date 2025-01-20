package frc.robot.SwerveUtils;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class SwerveWheel {
    private final SparkMax directionDriver;
    private final SparkMax speedDriver;
    private final CANcoder encoder;
    private final PIDController controller;
    private String WheelID;

    public SwerveWheel(SparkMax directionDriver, SparkMax speedDriver, CANcoder encoder, String wheelID, PIDController controller) {
        this.directionDriver = directionDriver;
        this.speedDriver = speedDriver;
        this.encoder = encoder;
        this.controller = controller;
        WheelID = wheelID;

    }

    public void setDirection(double setpoint){
        double currentAngle = getAbsoluteValue();
        controller.setSetpoint(currentAngle + closestAngle(currentAngle, setpoint));
        SmartDashboard.putNumber(WheelID.concat(" setpoint"), currentAngle + closestAngle(currentAngle, setpoint));
        directionDriver.set(controller.calculate(currentAngle));
        SmartDashboard.putNumber(WheelID.concat(" Direction"), controller.calculate(currentAngle));
    }

    public void stopDirection(){
        directionDriver.set(0);
    }

    public void setSpeed(double speed){
        speedDriver.set(speed);
    }

    public void stopSpeed(){
        speedDriver.stopMotor();
    }

    public double closestAngle(double a, double b)
    {
            // get direction
            double dir = modulo(b, 1.0) - modulo(a, 1.0);

            // convert from -360 to 360 to -180 to 180
            if (Math.abs(dir) > 0.5)
            {
                    dir = -(Math.signum(dir) * 1.0) + dir;
            }
            return dir;
    }

    private static double modulo(double x, double y){
        return x % y;
    }

    public double getAbsoluteValue(){
        return encoder.getAbsolutePosition().getValueAsDouble();
    }

    public void setSpeed(double power, boolean foward){
        power = power * 0.25;
        if(foward) speedDriver.set(power);
        else speedDriver.set(-power);

        if(power < 0.1){
            speedDriver.stopMotor();
        }
    }


}
