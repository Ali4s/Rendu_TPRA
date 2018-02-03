using UnityEngine;
using System.Collections;

using LockingPolicy = Thalmic.Myo.LockingPolicy;
using Pose = Thalmic.Myo.Pose;
using UnlockType = Thalmic.Myo.UnlockType;
using VibrationType = Thalmic.Myo.VibrationType;

// Change the material when certain poses are made with the Myo armband.
// Vibrate the Myo armband when a fist pose is made.
public class ColorBoxByPose : MonoBehaviour
{
    // Myo game object to connect with.
    // This object must have a ThalmicMyo script attached.
    public GameObject myo = null;
    public float speed = 0.0001f;

    // Materials to change to when poses are made.
    public Material waveInMaterial;
    public Material waveOutMaterial;
    public Material doubleTapMaterial;

    public bool canHold = true;

    public Transform guide;

    public GameObject cube;

    public raaaa boo;

    GameObject g;
    
    // The pose from the last update. This is used to determine if the pose has changed
    // so that actions are only performed upon making them rather than every frame during
    // which they are active.
    private Pose _lastPose = Pose.Unknown;

    private void Start()
    {
        boo = g.GetComponent<raaaa>();
    }

    // Update is called once per frame.
    void FixedUpdate ()
    {



        if (cube)
        {
            Rigidbody rbdy = cube.gameObject.GetComponent<Rigidbody>();

            //Stop Moving/Translating
            rbdy.velocity = Vector3.zero;

            //Stop rotating
            rbdy.angularVelocity = Vector3.zero;
        }
        

        // Access the ThalmicMyo component attached to the Myo game object.
        ThalmicMyo thalmicMyo = myo.GetComponent<ThalmicMyo> ();

        // Check if the pose has changed since last update.
        // The ThalmicMyo component of a Myo game object has a pose property that is set to the
        // currently detected pose (e.g. Pose.Fist for the user making a fist). If no pose is currently
        // detected, pose will be set to Pose.Rest. If pose detection is unavailable, e.g. because Myo
        // is not on a user's arm, pose will be set to Pose.Unknown.
        if (thalmicMyo.pose != _lastPose) {
            _lastPose = thalmicMyo.pose;

            // Vibrate the Myo armband when a fist is made.
            if (thalmicMyo.pose == Pose.Fist) {
              
                
                Pickup();

                ExtendUnlockAndNotifyUserAction(thalmicMyo);

            // Change material when wave in, wave out or double tap poses are made.
            } else if (thalmicMyo.pose == Pose.WaveIn) {
                GetComponent<Renderer>().material = waveInMaterial;

                ExtendUnlockAndNotifyUserAction (thalmicMyo);
            } else if (thalmicMyo.pose == Pose.WaveOut) {
                GetComponent<Renderer>().material = waveOutMaterial;

                ExtendUnlockAndNotifyUserAction (thalmicMyo);
            } else if (thalmicMyo.pose == Pose.DoubleTap) {
               
                Drop();
                ExtendUnlockAndNotifyUserAction (thalmicMyo);
            }
        }
    }

    void OnCollisionEnter(Collision col)
    {

        
        if (col.gameObject.name == "Cube")
        {        

            col.gameObject.GetComponent<Renderer>().material = waveInMaterial;

            cube = col.gameObject;               

        }

      

    }
    private void Pickup()
    { 
        
       Debug.Log("touch");

        if (!cube)
            return;

       
        //We set the object parent to our guide empty object.
        cube.transform.SetParent(guide);

        cube.GetComponent<Rigidbody>().useGravity = false;
        //we apply the same rotation our main object (Camera) has.

        cube.transform.position = guide.position;
        cube.transform.localRotation = guide.transform.rotation;
        //We re-position the ball on our guide object 
       

        canHold = false;
    }

    private void Drop()
    {
        if (!cube)
            return;

        //Set our Gravity to true again.
        // we don't have anything to do with our ball field anymore
        cube.GetComponent<Rigidbody>().useGravity = true;
        cube = null;
        //Apply velocity on throwing
         guide.GetChild(0).gameObject.GetComponent<Rigidbody>().velocity = transform.forward*speed;



        //Unparent our ball
        guide.GetChild(0).parent = null;
        canHold = true;
    }
    

    // Extend the unlock if ThalmcHub's locking policy is standard, and notifies the given myo that a user action was
    // recognized.
    void ExtendUnlockAndNotifyUserAction (ThalmicMyo myo)
    {
        ThalmicHub hub = ThalmicHub.instance;

        if (hub.lockingPolicy == LockingPolicy.Standard) {
            myo.Unlock (UnlockType.Timed);
        }

        myo.NotifyUserAction ();
    }
}
/*
 * 
 * using UnityEngine;
 using System.Collections;
 
 public class HoldItems : MonoBehaviour {
 
 
     public float speed = 10;
     public bool canHold = true;
     public GameObject ball;
     public Transform guide;
 
    void Update()
   {
       if (Input.GetMouseButtonDown(0))
       {
           if (!canHold)
               throw_drop();
           else
               Pickup();
       }//mause If
  
       if (!canHold && ball)
           ball.transform.position = guide.position;
       
   }//update
 
     //We can use trigger or Collision
     void OnTriggerEnter(Collider col)
     {
         if (col.gameObject.tag == "ball")
             if (!ball) // if we don't have anything holding
                 ball = col.gameObject;
     }
 
     //We can use trigger or Collision
     void OnTriggerExit(Collider col)
     {
         if (col.gameObject.tag == "ball")
         {
             if (canHold)
                 ball = null;
         }
     }
 
 
     private void Pickup()
     {
         if (!ball)
             return;
 
         //We set the object parent to our guide empty object.
         ball.transform.SetParent(guide);
 
         //Set gravity to false while holding it
         ball.GetComponent<Rigidbody>().useGravity = false;
 
         //we apply the same rotation our main object (Camera) has.
         ball.transform.localRotation = transform.rotation;
         //We re-position the ball on our guide object 
         ball.transform.position = guide.position;
 
         canHold = false;
     }
 
     private void throw_drop()
     {
         if (!ball)
             return;
 
         //Set our Gravity to true again.
         ball.GetComponent<Rigidbody>().useGravity = true;
          // we don't have anything to do with our ball field anymore
          ball = null; 
         //Apply velocity on throwing
         guide.GetChild(0).gameObject.GetComponent<Rigidbody>().velocity = transform.forward * speed;
 
         //Unparent our ball
         guide.GetChild(0).parent = null;
         canHold = true;
     }
 }//class
*/