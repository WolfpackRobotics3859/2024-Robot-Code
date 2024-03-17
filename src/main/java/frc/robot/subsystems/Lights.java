// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase 
{
  private final Timer m_LightTimer = new Timer();
  AddressableLEDBuffer m_ledBuffer1 = new AddressableLEDBuffer(115);
  AddressableLED m_led = new AddressableLED(2);
  int[] symbolLights = {3, 4, 5, 7, 8, 9, 11, 12, 13, 15, 16, 17, 28, 30, 32, 34, 38, 40, 47, 48, 49, 51, 52, 53, 55, 56, 57, 59, 60, 72, 74, 76, 80, 84, 91, 95, 96, 97, 99, 100, 101, 103, 104, 105};
  private int m_BrightnessCount;
  private boolean m_IsGoingUp = true;

  /**
   * @brief Creates a new Lights subsystem. 
   */
  public Lights() 
  {
    m_LightTimer.start();
    m_led.setLength(m_ledBuffer1.getLength());
  }

  @Override
  public void periodic() 
  {
    if (m_LightTimer.get() > 0.5)
    {
      m_LightTimer.reset();
    }
  }

  /**
   * @brief Sets the default color of the lights to orange.
   * @param hue The hue (color) of the lights.
   * @param saturation The saturation (amount of gray) in the lights.
   * @param brightness The brightness of the lights.
   */
  public void setOrange()
  {
    for (int i = 0; i < m_ledBuffer1.getLength(); i++) 
    {
      m_ledBuffer1.setHSV(i, 4, 255, 255);
    }
    m_led.setData(m_ledBuffer1);
    m_led.start();
  }

  /**
   * @brief Sets the color of the lights when it has a note to green.
   * @param hue The hue (color) of the lights.
   * @param saturation The saturation (amount of gray) in the lights.
   * @param brightness The brightness of the lights.
   */  
  public void setGreen()
  {
    for (int i = 0; i < m_ledBuffer1.getLength(); i++)
    {
      m_ledBuffer1.setHSV(i, 56, 255, 255);
    }
    m_led.setData(m_ledBuffer1);
    m_led.start();
  }

  /**
   * @brief Sets the color of the lights to blue.
   * @param hue The hue (color) of the lights.
   * @param saturation The saturation (amount of gray) in the lights.
   * @param brightness The brightness of the lights.
   */  
  public void setBlue()
  {
    for (int i = 0; i < m_ledBuffer1.getLength(); i++)
    {
      m_ledBuffer1.setHSV(i, 100, 255, 255);
    }
    m_led.setData(m_ledBuffer1);
    m_led.start();
  }

  /**
   * @brief Sets the color of the lights to red.
   * @param hue The hue (color) of the lights.
   * @param saturation The saturation (amount of gray) in the lights.
   * @param brightness The brightness of the lights.
   */  
  public void setRed()
  {
    for (int i = 0; i < m_ledBuffer1.getLength(); i++)
    {
      m_ledBuffer1.setHSV(i, 0, 255, 255);
    }
    m_led.setData(m_ledBuffer1);
    m_led.start();
  }

  /**
   * @brief Sets the color of the lights when intaking a note to flashing orange.
   * @param hue The hue (color) of the lights.
   * @param saturation The saturation (amount of gray) in the lights.
   * @param brightness The brightness of the lights.
   */    
  public void setOrangeFlash()
  {
    if (m_LightTimer.get() > .25)
      {
        setOffLights();
        m_led.setData(m_ledBuffer1);
      }
    else
      {
        for (int x = 0; x < m_ledBuffer1.getLength(); x++)
        {
          m_ledBuffer1.setHSV(x, 4, 255, 255);
        }
        m_led.setData(m_ledBuffer1);
      }
    m_led.start();
  }

  /**
   * @brief Sets the color of the lights to flashing blue.
   * @param hue The hue (color) of the lights.
   * @param saturation The saturation (amount of gray) in the lights.
   * @param brightness The brightness of the lights.
   */
  public void setBlueFlash()
  {
    if (m_LightTimer.get() > .25)
      {
        setOffLights();
        m_led.setData(m_ledBuffer1);
      }
    else
      {
        for (int x = 0; x < m_ledBuffer1.getLength(); x++)
        {
          m_ledBuffer1.setHSV(x, 100, 255, 255);
        }
        m_led.setData(m_ledBuffer1);
      }
    m_led.start();
  }

  /**
   * @brief Sets the color of the lights to flashing red.
   * @param hue The hue (color) of the lights.
   * @param saturation The saturation (amount of gray) in the lights.
   * @param brightness The brightness of the lights.
   */
  public void setRedFlash()
  {
    if (m_LightTimer.get() > .25)
      {
        setOffLights();
        m_led.setData(m_ledBuffer1);
      }
    else
      {
        for (int x = 0; x < m_ledBuffer1.getLength(); x++)
        {
          m_ledBuffer1.setHSV(x, 0, 255, 255);
        }
        m_led.setData(m_ledBuffer1);
      }
    m_led.start();
  }

  /**
   * @brief Sets the team number on the lights in orange on a blue background.
   * @param hue The hue (color) of the lights.
   * @param saturation The saturation (amount of gray) in the lights.
   * @param brightness The brightness of the lights.
   */
  public void setWolfpackSymbol()
  {
    for (int x = 0; x < m_ledBuffer1.getLength(); x++)
    {
      m_ledBuffer1.setHSV(x, 100, 230, 1);
    }
    for (int x = 0; x < symbolLights.length; x++)
    {
      m_ledBuffer1.setHSV(symbolLights[x], 4, 255, 255);
    }
    m_led.setData(m_ledBuffer1);
    m_led.start();
  }

  public void fadeInFadeOut(int hue, int saturation)
  {
    if(m_LightTimer.get() > 0.25)
    {
      if(m_IsGoingUp)
      {
        m_BrightnessCount += 10;
        if(m_BrightnessCount > 240)
        {
          m_IsGoingUp = false;
        }
      }
      else
      {
        m_BrightnessCount -= 10;
        if(m_BrightnessCount < 20)
        {
          m_IsGoingUp = true;
        }
      }
      for (int x = 0; x < m_ledBuffer1.getLength(); x++)
      {
        m_ledBuffer1.setHSV(x, hue, saturation, m_BrightnessCount);
      }
      m_led.setData(m_ledBuffer1);
    }
  }

  /**
   * @brief Turns off all of the lights when called.
   */
  public void setOffLights()
  {
    for (int x = 0; x < m_ledBuffer1.getLength(); x++)
    {
      m_ledBuffer1.setHSV(x, 0, 0, 0);
    }
  }
}