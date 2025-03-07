package frc.robot.SwerveUtils;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class SwerveWheel {
    public final SparkMax directionDriver;
    private final SparkMax speedDriver;
    private final CANcoder encoder;
    private final PIDController controller;
    public String WheelID;

    public SwerveWheel(SparkMax directionDriver, SparkMax speedDriver, CANcoder encoder, String wheelID,
            PIDController controller) {
        this.directionDriver = directionDriver;
        this.speedDriver = speedDriver;
        this.encoder = encoder;
        this.controller = controller;
        WheelID = wheelID;

    }

    public void setDirection(double setpoint) {
        double currentAngle = getAbsoluteValue();
        controller.setSetpoint(currentAngle + closestAngle(currentAngle, setpoint));

        SmartDashboard.putNumber(WheelID.concat(" setpoint"), currentAngle + closestAngle(currentAngle, setpoint));
        SmartDashboard.putNumber(WheelID.concat(" closestAngle"), closestAngle(currentAngle, setpoint));
        SmartDashboard.putNumber(WheelID.concat(" encoder value"), getAbsoluteValue());

        directionDriver.set(controller.calculate(currentAngle));
        SmartDashboard.putNumber(WheelID.concat(" Direction"), controller.calculate(currentAngle));
    }

    public void stopDirection() {
        directionDriver.set(0);
    }

    protected double closestAngle(double a, double b) {
        // get direction
        a = Units.rotationsToDegrees(a);
        b = Units.rotationsToDegrees(b);
        double dir = modulo(b, 360.0) - modulo(a, 360.0);

        // convert from -360 to 360 to -180 to 180
        if (Math.abs(dir) > 180.0) {
            dir = -(Math.signum(dir) * 360.0) + dir;
        }
        return Units.degreesToRotations(dir);
    }

    private static double modulo(double x, double y) {
        return x % y;
    }

    public double getAbsoluteValue() {
        return encoder.getAbsolutePosition().getValueAsDouble();
    }

    public Rotation2d getR2D() {
        return new Rotation2d(Units.rotationsToRadians(encoder.getAbsolutePosition().getValueAsDouble()));
    }

    public void setSpeed(double power) {
        power = power * Constants.MAX_SPEED;
        speedDriver.set(power);

        if (Math.abs(power) < 0.1) {
            speedDriver.stopMotor();
        }
    }

    public double getRotSpeedInSec(){
        return (speedDriver.getAlternateEncoder().getVelocity() * 60);
    }

    public double getMeters() {
        double metros = speedDriver.getEncoder().getPosition() * Constants.kWheelCircuferenceMeters;
        return metros;
    }
    
}
