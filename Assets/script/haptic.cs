using UnityEngine;
using System.IO.Ports;
using System.Threading;
using System;

public class haptic : MonoBehaviour
{
    static SerialPort sp = new SerialPort("\\\\.\\COM3", 115200);// make sure arduino file and this port value should be same and the baudrate as well
    //coz the rate at which you send data must be received at same rate or else you might face data Overflow
    bool threadActive = true;
    public Transform Elbo = null;
    public Transform Wrist = null;
    public Transform thumb = null;
    public Transform index = null;
    public Transform middle = null;
    public Transform pinky = null;
    public Transform ring = null;

    private Vector3 _Elbo;
    private Vector3 _ElboOffset = Vector3.zero;

    private Vector3 _Wrist;
    private Vector3 _WristOffset = Vector3.zero;

    private float bendthumb;
    private float bendindex;
    public float bendmiddle;
    private float bendpinky;
    private float bendring;

    int indexVib = 0;
    int middleVib = 0;
    int ringVib = 0;
    int thumbVib = 0;

    private const float _rate = .5f;

    private void setNodeRotation(Transform node, float angle)
    {
        var rotation = Quaternion.Euler(0, angle, 0); // +90 degrees Y rotation
        node.localRotation = Quaternion.Slerp(node.localRotation, rotation, _rate);
    }

    private void setThumbRotation(Transform thumb, float angle)
    {
        var rotation = Quaternion.Euler(5, 4, -angle);
        thumb.localRotation = Quaternion.Slerp(thumb.localRotation, rotation, _rate);
    }

    void Update()
    {
        setNodeRotation(pinky, -bendpinky);
        setNodeRotation(pinky.GetChild(0), -bendpinky);
        setNodeRotation(pinky.GetChild(0).GetChild(0), -bendpinky);

        setNodeRotation(ring, -bendring);
        setNodeRotation(ring.GetChild(0), -bendring);
        setNodeRotation(ring.GetChild(0).GetChild(0), -bendring);

        setNodeRotation(middle, -bendmiddle);
        setNodeRotation(middle.GetChild(0), -bendmiddle);
        setNodeRotation(middle.GetChild(0).GetChild(0), -bendmiddle);

        setNodeRotation(index, -bendindex);
        setNodeRotation(index.GetChild(0), -bendindex);
        setNodeRotation(index.GetChild(0).GetChild(0), -bendindex);


        setThumbRotation(thumb.GetChild(0), -bendthumb);
        setThumbRotation(thumb.GetChild(0).GetChild(0), -bendthumb);


        _Elbo = new Vector3(-_Elbo.x, _Elbo.y, -_Elbo.z);
        Elbo.localRotation = Quaternion.Euler(_Elbo + _ElboOffset);

        _Wrist = new Vector3(_Wrist.z, _Wrist.y, _Wrist.x);
        Wrist.localRotation = Quaternion.Euler(_Wrist + _WristOffset);
    }

    void Awake()
    {
        Thread serialReader = new Thread(SerialReadThread);
        serialReader.Start();
    }

    void SerialReadThread()
    {
        while (threadActive)
        {
            if (sp.IsOpen)
            {
                try
                {
                    //taking the series of value form serial and spiltting and storing in array at index
                    string[] output_array = sp.ReadLine().Split(',');

                    // using the mathematical function to get value in degree
                    // x=((Current value - Lower value)/(Higher value - Lower value))* angle of degree to rotate 
                    bendthumb = ((float.Parse(output_array[0]) - 580) / 28f) * 50f;
                    bendindex = ((float.Parse(output_array[1]) - 623) / 17f) * 45f;
                    bendmiddle = ((float.Parse(output_array[2]) - 547) / 15f) * 40f;
                    bendring = ((float.Parse(output_array[3]) - 583) / 24f) * 40f;//we used the same value of ring and little finger
                    bendpinky = ((float.Parse(output_array[3]) - 583) / 24f) * 40f;

                    //x,y,z value from 1st mpu6050 at elbo 
                    _Elbo.x = (float.Parse(output_array[4]));
                    _Elbo.y = (float.Parse(output_array[5]));
                    _Elbo.z = (float.Parse(output_array[6]));
                    
                    //x,y,z value from 2st mpu6050 at wrist
                    _Wrist.x = (float.Parse(output_array[7]));
                    _Wrist.y = (float.Parse(output_array[8]));
                    _Wrist.z = (float.Parse(output_array[9]));

                    //to send signal to arduino if object collide 
                    // we have ended the string with "~" and check the same in arduino code for end of string
                    sp.Write(indexVib.ToString() + middleVib.ToString() + ringVib.ToString() + thumbVib.ToString() + "~");

                    // Reset vibrations
                    indexVib = 0;
                    middleVib = 0;
                    ringVib = 0;
                    thumbVib = 0;

                }
                catch (System.Exception)
                {

                }
            }
            else
            {
                sp.Open(); // Open the serial port
                sp.ReadTimeout = 250; // Timeout for reading 
            }
        }
    }



    public void ReceiveTriggers(string colName, string fingerName)
    {
        if (colName.Equals("Cube1"))
        {

            switch (fingerName)
            {
                case "BoneIndexBase":
                    indexVib = 1;
                    break;
                case "BoneMiddleBase":
                    middleVib = 1;
                    break;
                case "BoneRingBase":
                    ringVib = 1;
                    break;
                case "Bone003":
                    thumbVib = 1;
                    break;
            }
        }
        else
        {
            // this part is if it touches different object you can modify the way moter vibrate
            switch (fingerName)
            {
                case "BoneIndexBase":
                    indexVib = 2;
                    break;
                case "BoneMiddleBase":
                    middleVib = 2;
                    break;
                case "BoneRingBase":
                    ringVib = 2;
                    break;
                case "Bone003":
                    thumbVib = 2;
                    break;
            }
        }
    }

    void OnApplicationQuit()
    {
        threadActive = false;
    }

}
