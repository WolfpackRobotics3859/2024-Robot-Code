package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public class RoboLogger 
{
    /**
     * @brief Tracks if the DataLog has been started or is stopped.
     */
    private boolean m_isEnabled;

    /**
     * @brief Creates a new RoboLogger and optionally starts the DataLog.
     * @param isEnabled True, begins the DataLog. False, doesn't begin the DataLog.
     */
    public RoboLogger(boolean isEnabled)
    {
        this.enableLogger(isEnabled);
    }

    /**
     * @brief Enables or disables the DataLog.
     * @param isEnabled True, begins the DataLog. False, stops the DataLog.
     */
    public void enableLogger(boolean isEnabled)
    {
        if(m_isEnabled != isEnabled)
        {
            if(isEnabled)
            {
                this.enableDataLog();
                m_isEnabled = true;
            }
            else
            {
                DataLogManager.stop();
                m_isEnabled = false;
            }
        }
    }

    /**
     * @brief Prints a system message with a UserID.
     * @param userId The caller's name.
     * @param message The message to send.
     */
    public void print(String userId, String message)
    {
        System.out.printf("[%f][%s] %s", Timer.getFPGATimestamp(), userId, message);
    }

    /**
     * @brief Enables the DataLog with known settings.
     */
    private void enableDataLog()
    {
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());
    }    
}
