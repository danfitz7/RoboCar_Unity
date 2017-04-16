#define USE_HAND_SNAPS

using System.Collections;
using UnityEngine;
using System.Collections.Generic;

using System.IO.Ports;
using System.IO;
using System;

using Vuforia;

using Consistent_Overhead_Byte_Stuffing;

public class RobotFollowDriver : MonoBehaviour, ITrackableEventHandler
{
	const float FOLLOWING_DISTANCE_M = 1.25f; // How far behind to follow the target, in meters

    
	#if UNITY_EDITOR_WINDOWS
		const string DEFAULT_ARDUINO_PORT_NAME = "\\\\.\\COM7";//"COM6";
	#elif UNITY_EDITOR_OSX
		const string DEFAULT_ARDUINO_PORT_NAME = "/dev/tty.usbmodem2729671";//"/dev/tty.usbmodem1421";
	#elif UNITY_ANDROID
		const string DEFAULT_ARDUINO_PORT_NAME = "/dev/bus/usb/001/002";
	#else
		const string DEFAULT_ARDUINO_PORT_NAME = "UNDEFINED DEFAULT PORT FOR THIS PLATFORM";
	#endif

    const int SERIAL_BAUD = 9600;
    const string ARDUINO_ACK = "ACK";
    const string DRIVE_ARDUINO_ADDRESS = "255";

 //   static readonly string DRIVE_COMMAND = "DRIVE";
    static readonly int DRIVE_MESSAGE_DATA_BYTES = (2 * 2); // One byte delimiter + 2 numbers, 2 bytes each
    const byte COBS_DELIMITER = 0;
    byte[] driveMessageData;
    byte[] COBSEncodedMessage = new byte[DRIVE_MESSAGE_DATA_BYTES + DRIVE_MESSAGE_DATA_BYTES/254 +1];
    const int DEFAULT_ARDIUNO_PORT_READ_TIMEOUT_MS = 50;
    const int DEFAULT_ARDIUNO_PORT_WRITE_TIMEOUT_MS = 16;   // 16.67ms between frames at 60fps
   
    System.IO.Ports.SerialPort arduinoSerialPort;
	string arduinoPortName = DEFAULT_ARDUINO_PORT_NAME;

	// For tracking calculations
	private TrackableBehaviour mTrackableBehaviour;
	bool tracking = false;
	GameObject ARCamera;

    // These are the actual values we are trying to convey to the arduino
    float distanceError_m = 0.0f;
    float angleError_deg = 0.0f;

	const float COMMUNICATION_INT16_MAX = 32767.0f;
	const float twoPi = (float)Math.PI * 2.0f; 

    // Shamelessly copied from https://forum.unity3d.com/threads/serial-communication-between-unity-5-and-arduino.331005/
    List<String> GetPortNames(){
        print("Printing serial port names...");
        List<string> serial_ports;

        // What OS are we running on?
        int p = (int)System.Environment.OSVersion.Platform;

        // If' we're running on mac or linux
        if (p == 4 || p == 128 || p == 6) { // 4 = linux, 6= mac, 128 = NA. See https://msdn.microsoft.com/en-us/library/3a8hyw88(v=vs.110).aspx
            //print("\tNot running on windows" + p);
            serial_ports = new List<string>();
            string[] ttys = Directory.GetFiles("/dev/", "tty.*");
            foreach (string dev in ttys) {
                if (dev.StartsWith("/dev/tty.*")) {
                    serial_ports.Add(dev);
                }
                Debug.Log(System.String.Format(dev));
            }
        }else{
            //print("\tRunning on Windows");
            serial_ports = new List<string>(System.IO.Ports.SerialPort.GetPortNames());
        }

        if (serial_ports.Count >= 1) {
            string portNamesList = "Serial ports are: ";
            foreach (string portName in serial_ports) {
                portNamesList += portName + "\t";
            }
            print(portNamesList);
        }else{
            print("No serial ports found!");
        }

        return serial_ports;
    }

    void sendAck(){
        print("Sending ACK...");
		if (arduinoSerialPort != null && arduinoSerialPort.IsOpen) {
			arduinoSerialPort.Write (ARDUINO_ACK);
			arduinoSerialPort.BaseStream.Flush ();
		}
    }

    // Open Serial Port connection to Arduino
    public void OpenArduinoSerialPortConnection(){
        print("Opening port " + arduinoPortName);
		arduinoSerialPort = new System.IO.Ports.SerialPort(arduinoPortName, SERIAL_BAUD);    // Connect to the driver Arduino
        if (arduinoSerialPort != null){
            if (arduinoSerialPort.IsOpen){
                arduinoSerialPort.Close();
                Debug.Log("Closing port, because it was already open!");
            }else{
                arduinoSerialPort.ReadTimeout = DEFAULT_ARDIUNO_PORT_READ_TIMEOUT_MS;
                arduinoSerialPort.WriteTimeout = DEFAULT_ARDIUNO_PORT_READ_TIMEOUT_MS;
                arduinoSerialPort.Open();
                Debug.Log("Port Opened!");
            }
        }else{
            if (arduinoSerialPort.IsOpen){
                print("Port is already open");
            }else{
                print("Port == null");
            }
        }
    }

    // Close any open serial port connection before terminating
	void OnApplicationQuit(){
		if (arduinoSerialPort != null && arduinoSerialPort.IsOpen) {
			arduinoSerialPort.Close ();
			Debug.Log ("Arduino Port closed!");
		}
    }

	/*
    // Arduino serial read asynch callback (doesn't clog up main loop)
    public IEnumerator AsynchronousReadFromArduino(Action<string> callback, Action fail = null, float timeout = float.PositiveInfinity){
        DateTime initialTime = DateTime.Now;
        DateTime nowTime;
        TimeSpan diff = default(TimeSpan);
        string dataString = null;
        do{
            try{
                dataString = arduinoSerialPort.ReadLine();
            }catch (TimeoutException){
                dataString = null;
            }

            if (dataString != null){
                callback(dataString);
                yield return null;
			}else{
                yield return new WaitForSeconds(0.05f);
			}
            nowTime = DateTime.Now;
            diff = nowTime - initialTime;

        }
		while (diff.Milliseconds < timeout);
		if (fail != null) {
			fail ();
		}
        yield return null;
    }
	*/

	void sendDriveCommand(){
		if (this.enabled) {

			//print("Sending drive command");
	        
			// Unity is in meters, so multiply by 10,000 to get 1mm as base unit
			float distErrorf = distanceError_m * 1000.0f;
			Int16 distError = (Int16)Math.Round(distErrorf);
			if (distErrorf > COMMUNICATION_INT16_MAX) {
				distError = (Int16)(COMMUNICATION_INT16_MAX);
			} else if (distError < -COMMUNICATION_INT16_MAX) {
				distError = (Int16)(-COMMUNICATION_INT16_MAX);
			}

			// Convert angle and ensure it is in [-180, 180] degrees
			float rotationErrorRadians = angleError_deg * Mathf.Deg2Rad;
			while (rotationErrorRadians <= -Math.PI) {    // If we're less than -180 degrees, add a rotation so we're in -180<r<180
				rotationErrorRadians += twoPi;
			}
			while (rotationErrorRadians >= Math.PI) {    // if we're more than +180 degrees, subtract a rotation so we're in -180<r<180
				rotationErrorRadians -= twoPi;
			}
			Int16 rotationErrorInt = (Int16)(Math.Round((rotationErrorRadians * COMMUNICATION_INT16_MAX) / Math.PI));

			// Good for debugging
			print ("D<" + distError + "mm\t" + ((((float)rotationErrorInt) * 180.0f) / 32767.0f).ToString () + "deg>");


			// Byte stuffing, yay!
			int dataStartIndex = 0;
			driveMessageData [dataStartIndex + 0] = (byte)(distError >> 8);
			driveMessageData [dataStartIndex + 1] = (byte)(distError);
			driveMessageData [dataStartIndex + 2] = (byte)(rotationErrorInt >> 8);
			driveMessageData [dataStartIndex + 3] = (byte)(rotationErrorInt);

			int encodededLength = COBS.cobs_encode (ref driveMessageData, DRIVE_MESSAGE_DATA_BYTES, ref COBSEncodedMessage);
			//        print("Encoded message (" + encodededLength + ")" + BitConverter.ToString(COBSEncodedMessage));

			byte[] finalMessage = new byte[encodededLength + 1];
			Buffer.BlockCopy (COBSEncodedMessage, 0, finalMessage, 1, encodededLength);
			finalMessage [0] = 0;

			//print("Initial message (6)" + BitConverter.ToString(driveMessageData) + "\nFinal message: " + BitConverter.ToString(finalMessage));

			if (arduinoSerialPort != null && arduinoSerialPort.IsOpen) {
				arduinoSerialPort.Write (finalMessage, 0, encodededLength + 1);
			}
			//arduinoSerialPort.BaseStream.Flush();
		}
    }



	// implement ITrackableEventHandler
	public void OnTrackingFound(){
		tracking = true;
	}
	public void OnTrackingLost(){
		tracking = false;

		// Stop as soon as we lose tracking
		zeroDrive();
	}
	public void OnTrackableStateChanged(TrackableBehaviour.Status previousStatus, TrackableBehaviour.Status newStatus){
		if (newStatus == TrackableBehaviour.Status.DETECTED ||
			newStatus == TrackableBehaviour.Status.TRACKED ||
			newStatus == TrackableBehaviour.Status.EXTENDED_TRACKED){
			OnTrackingFound();
		}else{
			OnTrackingLost();
		}
	}	

    void Start(){
        print("RoboCar Following Driver Starting...");

		ARCamera = GameObject.Find("ARCamera");
		if (ARCamera == null){
			print("ERROR: Could not find camera object");
		}

		driveMessageData = new byte[DRIVE_MESSAGE_DATA_BYTES];  // setup output message 

		// Register to receive OnTrackingFound and OnTrackingLost events
		mTrackableBehaviour = GetComponent<TrackableBehaviour>();
		if (mTrackableBehaviour){
			mTrackableBehaviour.RegisterTrackableEventHandler(this);
		}
			

        // TODO: filter port names to find arduinos, then connect to each of those, monitor it's messages for the drive teensy id number, and only connect to that arduino
		GetPortNames();
        OpenArduinoSerialPortConnection();
		sendAck();

        // Start reading from the arduino serial port
        //        StartCoroutine(
        //            AsynchronousReadFromArduino
        //            (   arduinoSerialReadCallback,                   // Callback
        //                () => Debug.LogError("Port Read Error!"),    // Error callback
        //               10f                                           // Timeout (seconds)
        //           )
        //       );

    }

	// TODO: implement an actual emergency stop command
    void zeroDrive() {
        // reset our position
        distanceError_m = 0.0f;
        angleError_deg = 0.0f;

        sendDriveCommand();
    }

    public string ReadFromArduino(int timeout = 0){
		if (arduinoSerialPort != null && arduinoSerialPort.IsOpen) {
			if (timeout > 0) {
				arduinoSerialPort.ReadTimeout = timeout;
			}
			try {
				//            if (arduinoSerialPort != null && arduinoSerialPort.IsOpen && arduinoSerialPort.BytesToRead > 0){ // If there are any bytes to read in the input buffer
				return arduinoSerialPort.ReadLine();
				//            }else{
				//                return null;
				//            }
			} catch (TimeoutException) {
				return null;
			}
		}
		return null;
    }

    void Update(){

        // Print anything received from the arduino.
        // TODO: setup async serial read
        string received = ReadFromArduino(1);
		if (received != null){// && received.Length > 5){
            print("R: " + received);
			if (received.Trim().Equals(DRIVE_ARDUINO_ADDRESS)){
				print("ACK");
                sendAck();
            }
        }

        // If we are tracking our marker in this frame, go towards it
        if (tracking) {
			// Here we actually calculate our distance and angle from the origin (camera) to ourselves
			// get the vector between the camera (this robot) and target (following tracking target)
			Vector3 cameraToTarget = this.transform.position - ARCamera.transform.position;

			distanceError_m = Mathf.Sqrt((cameraToTarget.x*cameraToTarget.x) + (cameraToTarget.z*cameraToTarget.z)); // TODO: won't work if tracked object is behind us..bust still tracked..somehow
			distanceError_m -= FOLLOWING_DISTANCE_M;
			angleError_deg = Mathf.Atan2(cameraToTarget.x, cameraToTarget.z) * Mathf.Rad2Deg;

        // If we can't find the marker, stop
        }

		// Whether we're tracking or not, send a command eiter way so the arduino knows we still care about it
		sendDriveCommand();
    }
}
