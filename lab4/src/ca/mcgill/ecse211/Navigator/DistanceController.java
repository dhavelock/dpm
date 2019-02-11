package ca.mcgill.ecse211.Navigator;

public class DistanceController extends Thread implements UltrasonicController {
  
  private static final int MAX_NUM_SAMPLES = 5;  // Number of samples to average over
 
  private int distance;
  private int numSamples;
  
  public DistanceController() {
    this.distance = 0;
    this.numSamples = 0;
  }
  
  public int getDistance() {
    return distance;
  }
  
  /**
   * processes distance provided by the US Sensor
   */
  @Override
  public void processUSData(int dist) {
    
    // normalize distance for reading greater than 255
    if (dist >= 255) {
      dist = 255;
    }
    
    // Take average of samples to mitigate any spikes in the value returned from US Sensor
    if (numSamples < MAX_NUM_SAMPLES) {
      this.distance = (numSamples*this.distance + dist) / (++numSamples);
    } else {
      this.distance = ((MAX_NUM_SAMPLES - 1)*this.distance + dist) / MAX_NUM_SAMPLES;
    }
        
  }
  
  @Override
  public int readUSDistance() {
    return this.distance;
  }
}
