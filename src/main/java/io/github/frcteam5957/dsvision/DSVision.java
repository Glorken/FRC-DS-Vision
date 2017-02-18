package io.github.frcteam5957.dsvision;

import java.util.ArrayList;
import edu.wpi.first.wpilibj.networktables.*;
import edu.wpi.first.wpilibj.tables.*;
import edu.wpi.cscore.*;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class DSVision {

	public static void main(String[] args) {

		// Loads our OpenCV library. This MUST be included
		System.loadLibrary("opencv_java310");

		// Connect NetworkTables, and get access to the publishing table
		NetworkTable.setClientMode();
		// Set your team number here
		NetworkTable.setTeam(5957);

		NetworkTable.initialize();

		NetworkTable publishingTable = NetworkTable.getTable("CameraPublisher");
		ITable retrotapeTable = publishingTable.getSubTable("Retrotape");
		ITable gearTable = publishingTable.getSubTable("Gear");
		// This is the network port you want to stream the raw received image to
		// By rules, this has to be between 1180 and 1190, so 1185 is a good
		// choice
		int streamPort = 1185;

		// This stores our reference to our mjpeg server for streaming the input
		// image
		MjpegServer inputStream = new MjpegServer("MJPEG Server", streamPort);

		// HTTP Camera
		// This is our camera name from the robot. this can be set in your robot
		// code with the following command
		// CameraServer.getInstance().startAutomaticCapture("YourCameraNameHere");
		// "USB Camera 0" is the default if no string is specified
		String cameraName = "Axis 5957";
		HttpCamera camera = setHttpCamera(cameraName, inputStream);
		// It is possible for the camera to be null. If it is, that means no
		// camera could
		// be found using NetworkTables to connect to. Create an HttpCamera by
		// giving a specified stream
		// Note if this happens, no restream will be created
		if (camera == null) {
			camera = new HttpCamera("Axis 5957", "http://10.59.57.19/axis-cgi/mjpg/video.cgi");
			inputStream.setSource(camera);
		}

		// This creates a CvSink for us to use. This grabs images from our
		// selected camera,
		// and will allow us to use those images in opencv
		CvSink imageSink = new CvSink("CV Image Grabber");
		imageSink.setSource(camera);

		// This creates a CvSource to use. This will take in a Mat image that
		// has had OpenCV operations
		// operations
		CvSource imageSource = new CvSource("CV Image Source", VideoMode.PixelFormat.kMJPEG, 640, 480, 30);
		MjpegServer cvStream = new MjpegServer("CV Image Stream", 1186);
		cvStream.setSource(imageSource);

		// All Mats and Lists should be stored outside the loop to avoid
		// allocations
		// as they are expensive to create
		Mat inputImage = new Mat();
		Mat hsv = new Mat();
		Mat streamed = new Mat();

		ArrayList<MatOfPoint> tapeContours;
		TapeFinder tapeFinder = new TapeFinder();

		ArrayList<MatOfPoint> gearContours;
		GearFinder gearFinder = new GearFinder();
		Scalar rectColor = new Scalar(0, 0, 0, 0);

		// Infinitely process image
		while (true) {

			hsv.copyTo(streamed);
			// Grab a frame. If it has a frame time of 0, there was an error.
			// Just skip and continue
			long frameTime = imageSink.grabFrame(inputImage);
			if (frameTime == 0)
				continue;

			// Below is where you would do your OpenCV operations on the
			// provided image
			// The sample below just changes color source to HSV
			Imgproc.cvtColor(inputImage, hsv, Imgproc.COLOR_BGR2HSV);

			tapeFinder.process(inputImage);
			tapeContours = tapeFinder.filterContoursOutput();
			try {
				Rect tapeOne = Imgproc.boundingRect(tapeContours.get(0));
				retrotapeTable.putNumber("Tape One Center", (tapeOne.x + (tapeOne.width / 2)));
			} catch (IndexOutOfBoundsException e) {
				retrotapeTable.putNumber("Tape One Center", -1);
			}
			try {
				Rect tapeTwo = Imgproc.boundingRect(tapeContours.get(1));
				retrotapeTable.putNumber("Tape Two Center", (tapeTwo.x + (tapeTwo.width / 2)));
			} catch (IndexOutOfBoundsException e) {
				retrotapeTable.putNumber("Tape Two Center", -1);
			}

			gearFinder.process(inputImage);
			gearContours = tapeFinder.filterContoursOutput();
			for (int i = 0; i < gearContours.size(); i++) {
				Rect r = Imgproc.boundingRect(gearContours.get(i));
				cvRectangle(streamed, new Point(r.x, r.y), new Point(r.x + r.width, r.y + r.height), rectColor, 2,
						Core.LINE_8, 0, streamed);
				gearTable.putNumber("Gear " + i + " Center", (r.x + (r.width / 2)));
			}

			// Here is where you would write a processed image that you want to
			// restreams
			// This will most likely be a marked up image of what the camera
			// sees
			// For now, we are just going to stream the HSV image
			imageSource.putFrame(streamed);
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

	private static void cvRectangle(Mat src, Point pt1, Point pt2, Scalar color, double thickness, int lineType,
			double shift, Mat dst) {
		src.copyTo(dst);
		if (color == null) {
			color = Scalar.all(1.0);
		}
		Imgproc.rectangle(dst, pt1, pt2, color, (int) thickness, lineType, (int) shift);
	}
}