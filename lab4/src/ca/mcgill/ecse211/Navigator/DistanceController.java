package ca.mcgill.ecse211.Navigator;

public class DistanceController extends Thread implements UltrasonicController {
  
  private static final int FILTER_OUT = 20;       // Number of times to check value when filtering
  private static final int MAX_NUM_SAMPLES = 5;  // Number of samples to average over
 
  private int distance;
  private int filterControl;
  private int numSamples;
  
  public DistanceController() {
    this.distance = 0;
    this.filterControl = 0;
    this.numSamples = 0;
  }
  
  public int getDistance() {
    return distance;
  }
  
  @Override
  public void processUSData(int dist) {
    
    // filter distance
//    if (distance >= 255 && filterControl < FILTER_OUT) {
//      filterControl++;
//    } else if (distance >= 255) {
//      this.distance = distance;
//    } else {
//      filterControl = 0;
//      this.distance = distance;
//    }
    
    if (dist >= 255) {
      dist = 255;
    }
    
    if (numSamples < MAX_NUM_SAMPLES) {
      this.distance = (numSamples*this.distance + dist) / (++numSamples);
    } else {
      this.distance = ((MAX_NUM_SAMPLES - 1)*this.distance + dist) / MAX_NUM_SAMPLES;
    }
    
    // System.out.println(dist + " " + this.distance);
    
  }
  
  @Override
  public int readUSDistance() {
    return this.distance;
  }
}
