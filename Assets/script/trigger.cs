using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class trigger : MonoBehaviour
{
    haptic hand;

    void Awake()
    {
        hand = transform.root.GetComponent<haptic>();
    }

    void OnTriggerStay(Collider col)
    {
        if (!col.tag.Equals("BoneHand"))
        {
            hand.ReceiveTriggers(col.name, gameObject.name);
        }
    }
}
