package Team4450.Robot23.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import Team4450.Lib.Util;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static Team4450.Robot23.Constants.*;

public class Arm extends SubsystemBase
{
    private CANSparkMax     motor = new CANSparkMax(WINCH_MOTOR, MotorType.kBrushless);
    private RelativeEncoder encoder = motor.getEncoder();
    private DigitalInput    limitSwitch = new DigitalInput(WINCH_SWITCH);

    private final double    ARM_MAX = 1000;

    public Arm()
    {
        Util.consoleLog();
    }

    public void setPower(double power)
    {
        // If power positive, which means retract, check limit switch stop if true.
        // If power negative, which means extend, check encoder for max height, stop if there.

        if ((power > 0 && limitSwitch.get()) || (power < 0 && encoder.getPosition() >= ARM_MAX)) power = 0;

        if (limitSwitch.get()) encoder.setPosition(0);

        motor.set(power);
    }

    public void stop()
    {
        motor.stopMotor();
    }

    public double getPosition()
    {
        return encoder.getPosition();
    }

    public boolean getSwitch()
    {
        return limitSwitch.get();
    }

    public void updateDS()
    {

    }
}
