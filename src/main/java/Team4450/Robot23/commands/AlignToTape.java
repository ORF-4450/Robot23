package Team4450.Robot23.commands;

import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import Team4450.Lib.SynchronousPID;
import Team4450.Lib.Util;
import Team4450.Robot23.subsystems.DriveBase;
import Team4450.Robot23.subsystems.PhotonVision;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AlignToTape extends CommandBase
{
    private PhotonVision            photonVision;
    private PhotonPipelineResult    result;
    private DriveBase               driveBase;
    private SynchronousPID          controller = new SynchronousPID("AlignToTape", 0.04, 0, 0);
    private final double            maxSpeed = .15, tolerance = 0.5;
    private double                  startTime, lastYaw;
    private boolean                 finished;


    public AlignToTape(PhotonVision photonVision, DriveBase driveBase)
    {
        this.photonVision = photonVision;

        controller.setOutputRange(-maxSpeed, maxSpeed);

        controller.setSetpoint(0);

        controller.setTolerance(tolerance);

        this.driveBase = driveBase;

        addRequirements(driveBase);
    }
    
    @Override
    public void initialize()
    {
        Util.consoleLog();

        // Select the reflective tape pipeline.
        photonVision.selectPipeline(0);

        photonVision.setLedMode(VisionLEDMode.kOn);

        startTime = Util.timeStamp();

        controller.reset();

        finished = false;
        lastYaw = Double.NaN;

        SmartDashboard.putBoolean("AutoTarget", true);
        SmartDashboard.putBoolean("TargetLocked", false);
    }

    @Override
    public void execute()
    {
        // Delay first test for targets to allow PV to get going after we turn on LED
        // and process the next camera image. Otherwise we call getLatestResult before
        // there is one available and result is null.

        if (Util.getElaspedTime(startTime) < 0.75) return;

        result = photonVision.getLatestResult();

        if (result.hasTargets())
        {
            PhotonTrackedTarget target = result.getBestTarget();

            lastYaw = target.getYaw();

            double speed = controller.calculate(lastYaw);

            if (controller.onTarget()) 
            {
                speed = 0;
                finished =  true;
                SmartDashboard.putBoolean("TargetLocked", true);
            }

            SmartDashboard.putNumber("Align Speed", speed);

            Util.consoleLog("yaw=%.2f  speed=%.4f", lastYaw, speed);

            driveBase.drive(0, -speed, 0);
        } else 
            finished = true;    // No targets visible.
    }

    @Override
    public boolean isFinished()
    {
        return finished;
    }

    @Override
    public void end(boolean interrupted) 
    {
        Util.consoleLog("interrupted=%b, last Yaw=%.2f", interrupted, lastYaw);

        driveBase.stop();

        //photonVision.setLedMode(VisionLEDMode.kOff);

        SmartDashboard.putBoolean("AutoTarget", false);
    }
}
