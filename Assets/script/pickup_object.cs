using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class pickup_object : MonoBehaviour
{
    public Transform dest;
    public haptic bend;
    private bool canhold = true;

    private GameObject grabbedObject = null;

    void OnTriggerEnter(Collider other)
    {
        // to grag object if its matches object tags and fingers value is greater than 35
        if (other.gameObject.CompareTag("Cube1") && canhold == true && bend.bendmiddle > 35)
        {
            grabbedObject = other.gameObject;
            grab(other);
        }
        else if (other.gameObject.CompareTag("Cube2") && canhold == true && bend.bendmiddle > 35)
        {
            grabbedObject = other.gameObject;
            grab(other);
        }
    }

    //function to grab object
    void grab(Collider collide)
    {
        collide.GetComponent<Rigidbody>().useGravity = false;
        collide.GetComponent<Rigidbody>().isKinematic = true;
        collide.transform.parent = GameObject.Find("Destination").transform;
        collide.transform.position = dest.position;
        canhold = false;
    }


    void FixedUpdate()
    {
        if (grabbedObject != null)
        {   // to ungrab object if its value is less than 35 and matches with object tag 
            if (grabbedObject.gameObject.CompareTag("Cube1") && canhold == false && bend.bendmiddle < 35)
            {
                grabbedObject.transform.parent = null;
                grabbedObject.GetComponent<Rigidbody>().useGravity = true;
                grabbedObject.GetComponent<Rigidbody>().isKinematic = false;
                canhold = true;

            }
            else if (grabbedObject.gameObject.CompareTag("Cube2") && canhold == false && bend.bendmiddle < 35)
            {
                grabbedObject.transform.parent = null;
                grabbedObject.GetComponent<Rigidbody>().useGravity = true;
                grabbedObject.GetComponent<Rigidbody>().isKinematic = false;
                canhold = true;
            }
        }
    }

}