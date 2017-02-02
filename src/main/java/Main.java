import java.util.ArrayList;

import edu.wpi.first.wpilibj.networktables.*;
import edu.wpi.first.wpilibj.tables.*;
import edu.wpi.cscore.*;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

public class Main {

	static final Scalar MIN_HSV = new Scalar(63.216, 0.0, 193.253);
	static final Scalar MAX_HSV = new Scalar(94.886, 33.940, 255.0);

	static final double MIN_CONTOUR_AREA = 100.0;
  public static void main(String[] args) {
    // Loads our OpenCV library. This MUST be included
    System.loadLibrary("opencv_java320");

    // Connect NetworkTables, and get access to the publishing table
    NetworkTable.setClientMode();
    // Set your team number here
    NetworkTable.setTeam(9999);

    NetworkTable.initialize();


    // This is the network port you want to stream the raw received image to
    // By rules, this has to be between 1180 and 1190, so 1185 is a good choice
    int streamPort = 1185;

    // This stores our reference to our mjpeg server for streaming the input image
    MjpegServer inputStream = new MjpegServer("MJPEG Server", streamPort);

    // Selecting a Camera
    // Uncomment one of the 2 following camera options
    // The top one receives a stream from another device, and performs operations based on that
    // On windows, this one must be used since USB is not supported
    // The bottom one opens a USB camera, and performs operations on that, along with streaming
    // the input image so other devices can see it.

    // HTTP Camera
    /*
    // This is our camera name from the robot. this can be set in your robot code with the following command
    // CameraServer.getInstance().startAutomaticCapture("YourCameraNameHere");
    // "USB Camera 0" is the default if no string is specified
    String cameraName = "USB Camera 0";
    HttpCamera camera = setHttpCamera(cameraName, inputStream);
    // It is possible for the camera to be null. If it is, that means no camera could
    // be found using NetworkTables to connect to. Create an HttpCamera by giving a specified stream
    // Note if this happens, no restream will be created
    if (camera == null) {
      camera = new HttpCamera("CoprocessorCamera", "YourURLHere");
      inputStream.setSource(camera);
    }
    */
    
      

    /***********************************************/

    // USB Camera
    
    // This gets the image from a USB camera 
    // Usually this will be on device 0, but there are other overloads
    // that can be used
    UsbCamera camera = setUsbCamera(0, inputStream);
    // Set the resolution for our camera, since this is over USB
    camera.setResolution(320,240);

    // This creates a CvSink for us to use. This grabs images from our selected camera, 
    // and will allow us to use those images in opencv
    CvSink imageSink = new CvSink("CV Image Grabber");
    imageSink.setSource(camera);

    // This creates a CvSource to use. This will take in a Mat image that has had OpenCV operations
    // operations 
    CvSource imageSource = new CvSource("CV Image Source", VideoMode.PixelFormat.kMJPEG, 640, 480, 30);
    MjpegServer cvStream = new MjpegServer("CV Image Stream", 1186);
    cvStream.setSource(imageSource);

    // All Mats and Lists should be stored outside the loop to avoid allocations
    // as they are expensive to create
    Mat inputImage = new Mat();
    Mat hsv = new Mat();
	ArrayList<MatOfPoint> contours = new ArrayList<>();

    // Infinitely process image
    while (true) {
      // Grab a frame. If it has a frame time of 0, there was an error.
      // Just skip and continue
      long frameTime = imageSink.grabFrame(inputImage);
      if (frameTime == 0) continue;

	  contours.clear();

      // Below is where you would do your OpenCV operations on the provided image
      // The sample below just changes color source to HSV
      Imgproc.cvtColor(inputImage, hsv, Imgproc.COLOR_BGR2HSV);
	  inputImage = threshold(inputImage);
	  contours = findContours(inputImage);
	  contours = filterContours(contours);
	  contours = convexHull(contours);
	  hsv = drawHook(contours);


      // Here is where you would write a processed image that you want to restreams
      // This will most likely be a marked up image of what the camera sees
      // For now, we are just going to stream the HSV image
      imageSource.putFrame(hsv);
    }
  }

  private static HttpCamera setHttpCamera(String cameraName, MjpegServer server) {
    // Start by grabbing the camera from NetworkTables
    NetworkTable publishingTable = NetworkTable.getTable("CameraPublisher");
    // Wait for robot to connect. Allow this to be attempted indefinitely
    while (true) {
      try {
        if (publishingTable.getSubTables().size() > 0) {
          break;
        }
        Thread.sleep(500);
        } catch (Exception e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }


    HttpCamera camera = null;
    if (!publishingTable.containsSubTable(cameraName)) {
      return null;
    }
    ITable cameraTable = publishingTable.getSubTable(cameraName);
    String[] urls = cameraTable.getStringArray("streams", null);
    if (urls == null) {
      return null;
    }
    ArrayList<String> fixedUrls = new ArrayList<String>();
    for (String url : urls) {
      if (url.startsWith("mjpg")) {
        fixedUrls.add(url.split(":", 2)[1]);
      }
    }
    camera = new HttpCamera("CoprocessorCamera", fixedUrls.toArray(new String[0]));
    server.setSource(camera);
    return camera;
  }

  private static UsbCamera setUsbCamera(int cameraId, MjpegServer server) {
    // This gets the image from a USB camera 
    // Usually this will be on device 0, but there are other overloads
    // that can be used
    UsbCamera camera = new UsbCamera("CoprocessorCamera", cameraId);
    server.setSource(camera);
    return camera;
  }

  private static Mat threshold(Mat input) {
	  Mat ret = new Mat();
	  Core.inRange(input, MIN_HSV, MAX_HSV, ret);
	  return ret;
  }

  private static ArrayList<MatOfPoint> findContours(Mat input) {
	  ArrayList<MatOfPoint> ret = new ArrayList<>();
	  Mat hierarchy = new Mat();
	  Imgproc.findContours(input, ret, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
	  return ret;
  }

  private static ArrayList<MatOfPoint> filterContours(ArrayList<MatOfPoint> contours) {
	  ArrayList<MatOfPoint> ret = new ArrayList<>();

	  for (MatOfPoint m : contours) {
		  double area = Imgproc.contourArea(m);
		  if (area >= MIN_CONTOUR_AREA) {
			  ret.add(m);
		  }
	  }

	  return ret;
  }

  private static ArrayList<MatOfPoint> convexHull(ArrayList<MatOfPoint> contours) {
	  ArrayList<MatOfPoint> ret = new ArrayList<>();
	  MatOfInt hull = new MatOfInt();
	  for (MatOfPoint m : contours) {
		  MatOfPoint temp = new MatOfPoint();
		  Imgproc.convexHull(m, hull);
		  temp.create((int) hull.size().height, 1, CvType.CV_32SC2);
		  for (int i = 0; i < hull.size().height; i++) {
			  int index = (int) hull.get(i, 0)[0];
			  double[] point = new double[] {m.get(index, 0)[0], m.get(index, 0)[1]};
			  temp.put(i, 0, point);
		  }
		  ret.add(temp);
	  }
	  return ret;
  }

  private static Mat drawHook(ArrayList<MatOfPoint> contours) {
	  Mat ret = new Mat();
	  Imgproc.drawContours(ret, contours, -1, new Scalar(255, 255, 255));
	  return ret;
  }
}