/*
 * OdometryCorrection.java
 */
package ca.mcgill.ecse211.odometer;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.hardware.ev3.LocalEV3;
import lejos.robotics.SampleProvider;
import lejos.hardware.Sound;

public class OdometryCorrection implements Runnable {
  private static final long CORRECTION_PERIOD = 10;
  private static final double TILE_LENGTH = 30.48;
  private static final double LIGHT_THRESHOLD = 0.10;
  private static final double START_X = -15.0;
  private static final double START_Y = -15.0;
  
  private Odometer odometer;

  private static Port lsPort = LocalEV3.get().getPort("S1");
  private SampleProvider sampleProvider;
  private SensorModes lightSensor;
  private float[] lsData;
  private int numXLines;
  private int numYLines;

  /**
   * This is the default class constructor. An existing instance of the odometer is used. This is to
   * ensure thread safety.
   * 
   * @throws OdometerExceptions
   */
  public OdometryCorrection() throws OdometerExceptions {

    this.odometer = Odometer.getOdometer();
    this.lightSensor = new EV3ColorSensor(lsPort);
	this.sampleProvider = lightSensor.getMode("Red");
	this.lsData = new float[lightSensor.sampleSize()];
	this.numXLines = 0;
	this.numYLines = 0;

  }

  /**
   * Here is where the odometer correction code should be run.
   * 
   * @throws OdometerExceptions
   */
  // run method (required for Thread)
  public void run() {
    long correctionStart, correctionEnd;
    float colourIntensity;
    double theta;
    
    odometer.setXYT(START_X, START_Y, 0);

    while (true) {
      correctionStart = System.currentTimeMillis();

      // TODO Trigger correction (When do I have information to correct?)
      // TODO Calculate new (accurate) robot position
      // TODO Update odometer with new calculated (and more accurate) vales
      
      sampleProvider.fetchSample(lsData, 0);
      colourIntensity = lsData[0];
      double[] odoData = odometer.getXYT();
      theta = odoData[2];
      
            
      if (colourIntensity <= LIGHT_THRESHOLD) {
    	  Sound.beep();
    	  
    	  if (theta <= 20 || theta >= 340) {
    		  odometer.setY(numYLines*TILE_LENGTH);
    		  numYLines++;
    	  } else if (theta >= 70 && theta <= 110) {
    		  odometer.setX(numXLines*TILE_LENGTH);
    		  numXLines++;
    	  } else if (theta >= 160 && theta <= 200) {
    		  numYLines--;
    		  odometer.setY(numYLines*TILE_LENGTH);
    	  } else if (theta >= 250 && theta <= 290) {
    		  numXLines--;
    		  odometer.setX(numXLines*TILE_LENGTH);
    	  }
    	  
      }
      
      

      // this ensure the odometry correction occurs only once every period
      correctionEnd = System.currentTimeMillis();
      if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
        try {
          Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
        } catch (InterruptedException e) {
          // there is nothing to be done here
        }
      }
    }
  }
}
