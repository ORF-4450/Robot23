package Team4450.Robot23.commands;

import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import Team4450.Lib.Util;
import Team4450.Robot23.subsystems.PhotonVision;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AlignToTape extends CommandBase
{
    private PhotonVision            photonVision;
    private PhotonPipelineResult    result;

    public AlignToTape(PhotonVision photonVision)
    {
        this.photonVision = photonVision;
    }
    
    @Override
    public void initialize()
    {
        Util.consoleLog();

        // Select the reflective tape pipeline.
        photonVision.selectPipeline(0);

        photonVision.setLedMode(VisionLEDMode.kOn);

        SmartDashboard.putBoolean("AutoTarget", true);
    }

    @Override
    public void execute()
    {
        result = photonVision.getLatestResult();

        if (result.hasTargets())
        {
            PhotonTrackedTarget target = result.getBestTarget();
        }
    }

    @Override
    public boolean isFinished()
    {
        return result.hasTargets();
    }

    @Override
    public void end(boolean interrupted) 
    {
        Util.consoleLog("interrupted=%b", interrupted);

        photonVision.setLedMode(VisionLEDMode.kOff);

        SmartDashboard.putBoolean("AutoTarget", false);
    }
}
