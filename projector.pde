import ipcapture.*;
import gab.opencv.*;

IPCapture cam;
PImage camera_image;

static final int STATE_START = 0, STATE_FREEZE_1 = 1, STATE_CHECK_1 = 2, STATE_FREEZE_2 = 3, STATE_CHECK_2 = 4;
int state = STATE_START;


String ROS_TOPIC = "/input_image";

void setup() {
  size(640,480);
    
  cam = new IPCapture(this, "http://localhost:8080/stream?topic="+ROS_TOPIC+"&height=480&widht=640&type=mjpeg", "", "");
  cam.start();
  cam.pixelWidth = 640;
  cam.pixelHeight = 480;
  
  // this works as well:
  
  // cam = new IPCapture(this);
  // cam.start("url", "username", "password");
  
  // It is possible to change the MJPEG stream by calling stop()
  // on a running camera, and then start() it with the new
  // url, username and password.
}

void draw() {
  if (cam.isAvailable()) {
    cam.read();
    camera_image = cam.get();
    
    // load level image
    PImage level_image = loadImage("levels/1.png");
    
    // blur and threshold
    camera_image.filter(BLUR, 6);
    camera_image.filter(THRESHOLD, 0.5);

    // find difference
    OpenCV opencv_level_image = new OpenCV(this, level_image);
    opencv_level_image.diff(camera_image);
    PImage grayDiff1 = opencv_level_image.getSnapshot();
    OpenCV opencv_camera_image = new OpenCV(this, camera_image);
    opencv_camera_image.diff(level_image);
    PImage grayDiff2 = opencv_camera_image.getSnapshot();
    
    PImage res = grayDiff1;
    res.loadPixels();
    for(int i=0; i<cam.pixelWidth*cam.pixelHeight; i++)
    {
      float val1 = level_image.pixels[i];
      float val2 = camera_image.pixels[i];
      int val_diff = round(abs( val1 - val2 ));
      res.pixels[i] = val_diff;
    }
    
    //println(average_color);
    
    image(res, 0, 0);
  }
}

void keyPressed() {
  if (key == ' ') {
    if (cam.isAlive()) cam.stop();
    else cam.start();
  }
}
