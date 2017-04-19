using UnityEngine;
using System.Collections;
using System.IO.Ports;

public class FPSDisplay : MonoBehaviour
{

	//http://answers.unity3d.com/questions/152035/printtogui.html
	static string myLog;
	private string output = "";
	private string stack = "";
	void OnEnable () {
		Application.RegisterLogCallback (HandleLog);
	}
	void OnDisable () {
		// Remove callback when object goes out of scope
		Application.RegisterLogCallback(null);
	}
	void HandleLog (string logString, string stackTrace, LogType type) {
		output = logString;
		stack = stackTrace;
		myLog = output + "\n" + myLog;
		if (myLog.Length > 5000){
			myLog = myLog.Substring(0, 4000);
		}
	}


	float deltaTime = 0.0f;

	void Update()
	{
		deltaTime += (Time.deltaTime - deltaTime) * 0.1f;
	}

	void OnGUI()
	{
		//http://answers.unity3d.com/questions/152035/printtogui.html
		//if (!Application.isEditor){ //Do not display in editor ( or you can use the UNITY_EDITOR macro to also disable the rest
			myLog = GUI.TextArea(new Rect(10, 10, Screen.width - 10, Screen.height - 10), myLog);
		//}

		int w = Screen.width, h = Screen.height;
		const int sizePercent = 4;
		GUIStyle style = new GUIStyle();

		Rect rect = new Rect(0, 0, w, h * sizePercent / 100);
		style.alignment = TextAnchor.UpperLeft;
		style.fontSize = h * sizePercent / 100;
		style.normal.textColor = new Color (0.0f, 0.0f, 0.5f, 1.0f);
		float msec = deltaTime * 1000.0f;
		float fps = 1.0f / deltaTime;
		string text = string.Format("{0:0.0} ms ({1:0.} fps)", msec, fps);
		GUI.Label(rect, text, style);
	}
}
