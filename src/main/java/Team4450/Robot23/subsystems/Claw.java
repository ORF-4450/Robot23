package Team4450.Robot23.subsystems;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import Team4450.Lib.FXEncoder;
import Team4450.Lib.Util;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static Team4450.Robot23.Constants.*;

public class Claw extends SubsystemBase
{
    private WPI_TalonFX     motor = new WPI_TalonFX(CLAW_MOTOR);
    private FXEncoder       encoder = new FXEncoder(motor);

    private final double    CLAW_MAX = 15000;

    public Claw()
    {
        Util.consoleLog();

        motor.setInverted(true);

        encoder.setInverted(true);

        // Enable automatic application of limit switches connected to the TalonFX controller.
        // The JST wire used to connect the switches to the controller is wired as follows:
        // black = forward switch, red = forward ground, white = reverse ground, yellow =
        // reverse switch. Once enabled the controller automatically obeys the switches.

        motor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
                                             LimitSwitchNormal.NormallyOpen,
                                             30);

        motor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
                                             LimitSwitchNormal.NormallyOpen,
                                             30);
    }

    /**
     * Sets claw motor power.
     * @param power + means open claw, - means close claw.
     */
    public void setPower(double power)
    {
        // If power positive, which means open, check limit switch stop if true.
        // If power negative, which means close, check encoder for max height, stop if there.

        //if ((power > 0 && limitSwitch.get()) || (power < 0 && encoder.get() >= ARM_MAX)) power = 0;

        //if (limitSwitch.get()) encoder.reset();

        power = Util.clampValue(power, .20);
        
        motor.set(power);
   }

    public void stop()
    {
        motor.stopMotor();
    }

    public int getPosition()
    {
        return encoder.get();
    }

    /**
     * Reset Claw encoder to zero.
     */
    public void resetPosition()
    {
        encoder.reset();
    }

    /**
     * Returns claw forward switch.
     * @return True when claw fully closed.
     */
    public boolean getClosedSwitch()
    {
        if (motor.isFwdLimitSwitchClosed() == 1)
            return true;
        else
            return false;
    }

    /**
     * Returns claw reverse switch.
     * @return True when claw fully open.
     */
    public boolean getOpenSwitch()
    {
        if (motor.isRevLimitSwitchClosed() == 1)
            return true;
        else
            return false;
    }

    public void updateDS()
    {

    }
}
