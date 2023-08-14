package Team4450.Robot23.subsystems;

import static Team4450.Robot23.Constants.*;

import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import Team4450.Lib.Util;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVision extends SubsystemBase
{
    private PhotonCamera    camera = new PhotonCamera("4450-LL");
    
    private VisionLEDMode   ledMode = VisionLEDMode.kOff;

	public PhotonVision() 
	{
        setLedMode(ledMode);

		Util.consoleLog("PhotonVision created!");
	}

    /**
     * Get the lastest target results object returned by the camera.
     * @return Results object.
     */
    public PhotonPipelineResult getLatestResult()
    {
        return camera.getLatestResult();
    }
 
    /**
     * Select camera's image processing pipeline.
     * @param index Zero based number of desired pipeline.
     */
    public void selectPipeline(int index)
    {
        camera.setPipelineIndex(index);
    }

    /**
     * Set the LED mode.
     * @param mode Desired LED mode.
     */
    public void setLedMode(VisionLEDMode mode)
    {
        camera.setLED(mode);
    }

    /**
     * Toggle LED mode on/off.
     */
    public void toggleLedMode()
    {
        if (ledMode == VisionLEDMode.kOff)
            ledMode = VisionLEDMode.kOn;
        else
            ledMode = VisionLEDMode.kOff;
        
        setLedMode(ledMode);
    }

    /**
     * Save pre-processed image from camera stream.
     */
    public void inputSnapshot()
    {
        camera.takeInputSnapshot();
    }

    /**
     * Save post-processed image from camera stream.
     */    public void outputSnapshot()
    {
        camera.takeOutputSnapshot();
    }
        
    @Override
	public void initSendable( SendableBuilder builder )
	{
        //super.initSendable(builder);
        builder.setSmartDashboardType("Subsystem");

        PhotonPipelineResult result = getLatestResult();

        builder.addBooleanProperty("has Targets", () -> result.hasTargets(), null);

        if (result.hasTargets())
        {
            PhotonTrackedTarget target = result.getBestTarget();

            builder.addDoubleProperty("target yaw", () -> target.getYaw(), null);
            builder.addDoubleProperty("target area", () -> target.getArea(), null);
        } else {
            builder.addDoubleProperty("target yaw", () -> 0, null);
            builder.addDoubleProperty("target area", () -> 0, null);
        }
	}   	
}
