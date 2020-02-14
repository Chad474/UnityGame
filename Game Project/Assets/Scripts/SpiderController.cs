using DitzelGames.FastIK;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SpiderController : MonoBehaviour
{
    // The target we are going to track
    [SerializeField] LayerMask groundLayer;
    [SerializeField] Transform target;
    [SerializeField] LegStepper frontLeftLegStepper;
    [SerializeField] LegStepper frontRightLegStepper;
    [SerializeField] LegStepper backLeftLegStepper;
    [SerializeField] LegStepper backRightLegStepper;
    
    //[SerializeField] LegStepper[] legStepper;
    
    [SerializeField] GameObject frontLeftFoot;
    [SerializeField] GameObject frontRightFoot;
    [SerializeField] GameObject backLeftFoot;
    [SerializeField] GameObject backRightFoot;
    

    // How fast we can turn and move full throttle
    [SerializeField] float turnSpeed;
    public float moveSpeed;
    // How fast we will reach the above speeds
    [SerializeField] float turnAcceleration;
    [SerializeField] float moveAcceleration;
    // Try to stay in this range from the target
    [SerializeField] float minDistToTarget;
    [SerializeField] float maxDistToTarget;
    // If we are above this angle from the target, start turning
    [SerializeField] float maxAngToTarget;

    // World space velocity
    Vector3 currentVelocity;
    // We are only doing a rotation around the up axis, so we only use a float here
    float currentAngularVelocity;
    public bool debug;

    public bool grounded;


    public int isEnabled = 0;

    void LateUpdate()
    {
        Ground();
        ApplyGravity();

        RootMotionUpdate();
    }

    public void increaseIsEnabled()
    {
        isEnabled++;
    }
    public int getIsEnabled()
    {
        return isEnabled;
    }

    public void startCo()
    {
        FastIKFabric s1 = frontLeftFoot.GetComponent<FastIKFabric>();
        s1.Target = frontLeftLegStepper.transform;
        frontLeftLegStepper.SetHomeTransformPosition(frontLeftLegStepper.transform);
        frontLeftLegStepper.SetGroundLayer(groundLayer);

        FastIKFabric s2 = frontRightFoot.GetComponent<FastIKFabric>();
        s2.Target = frontRightLegStepper.transform;
        frontRightLegStepper.SetHomeTransformPosition(frontRightLegStepper.transform);
        frontRightLegStepper.SetGroundLayer(groundLayer);

        FastIKFabric s3 = backLeftFoot.GetComponent<FastIKFabric>();
        s3.Target = backLeftLegStepper.transform;
        backLeftLegStepper.SetHomeTransformPosition(backLeftLegStepper.transform);
        backLeftLegStepper.SetGroundLayer(groundLayer);

        FastIKFabric s4 = backRightFoot.GetComponent<FastIKFabric>();
        s4.Target = backRightLegStepper.transform;
        backRightLegStepper.SetHomeTransformPosition(backRightLegStepper.transform);
        backRightLegStepper.SetGroundLayer(groundLayer);

        StartCoroutine(LegUpdateCoroutine());
    }

    // Only allow diagonal leg pairs to step together
    IEnumerator LegUpdateCoroutine()
    {
        // Run forever
        while (isEnabled == 4)
        {
            // Try moving one diagonal pair of legs
            do
            {
                frontLeftLegStepper.TryMove();
                backRightLegStepper.TryMove();
                // Wait a frame
                yield return null;

                // Stay in this loop while either leg is moving.
                // If only one leg in the pair is moving, the calls to TryMove() will let
                // the other leg move if it wants to.
            } while (backRightLegStepper.Moving || frontLeftLegStepper.Moving);

            // Do the same thing for the other pair
            do
            {
                frontRightLegStepper.TryMove();
                backLeftLegStepper.TryMove();
                yield return null;
            } while (backLeftLegStepper.Moving || frontRightLegStepper.Moving);
        }
    }
    void RootMotionUpdate()
    {
        // Get the direction toward our target
        Vector3 towardTarget = target.position - transform.position;
        // Vector toward target on the local XZ plane
        Vector3 towardTargetProjected = Vector3.ProjectOnPlane(towardTarget, transform.up);
        // Get the angle from the forward direction to the direction toward our target
        // Here we get the signed angle around the up vector so we know which direction to turn in
        float angToTarget = Vector3.SignedAngle(transform.forward, towardTargetProjected, transform.up);

        float targetAngularVelocity = 0;

        // If we are within the max angle (i.e. approximately facing the target)
        // leave the target angular velocity at zero
        if (Mathf.Abs(angToTarget) > maxAngToTarget)
        {
            // Angles in Unity are clockwise, so a positive angle here means to our right
            if (angToTarget > 0)
            {
                targetAngularVelocity = turnSpeed;
            }
            // Invert angular speed if target is to our left
            else
            {
                targetAngularVelocity = -turnSpeed;
            }
        }

        // Use our smoothing function to gradually change the velocity
        currentAngularVelocity = Mathf.Lerp( currentAngularVelocity, targetAngularVelocity,
            1 - Mathf.Exp(-turnAcceleration * Time.deltaTime));

        // Rotate the transform around the Y axis, 
        // making sure to multiply by delta time to get a consistent angular velocity
        transform.Rotate(0, Time.deltaTime * currentAngularVelocity, 0, Space.Self);

        // Movement
        Vector3 targetVelocity = Vector3.zero;

        // Don't move if we're facing away from the target, just rotate in place
        if (Mathf.Abs(angToTarget) < maxAngToTarget)
        {
            float distToTarget = Vector3.Distance(transform.position, target.position);

            // If we're too far away, approach the target
            if (distToTarget > maxDistToTarget)
            {
                targetVelocity = moveSpeed * transform.forward;
                //targetVelocity = moveSpeed * towardTargetProjected.normalized;
            }
            // If we're too close, reverse the direction and move away
            else if (distToTarget < minDistToTarget)
            {
                targetVelocity = moveSpeed * -transform.forward; //-towardTargetProjected.normalized;
            }
        }

        currentVelocity = Vector3.Lerp(currentVelocity, targetVelocity, 
            1 - Mathf.Exp(-moveAcceleration * Time.deltaTime));

        // Apply the velocity
        transform.position += currentVelocity * Time.deltaTime;
    }   
    
    #region Grounding

    void Ground()
    {
        var rayDown = new Ray(transform.position + (transform.up * 0.5f), -transform.up);
        var rayDiagonal = new Ray(transform.position, transform.TransformDirection(0, -1, 1));
        //var rayDiagonalNegative = new Ray(transform.position + (transform.up * 0.05f), transform.TransformDirection(0,-1,-1));
        var hitInfo = new RaycastHit();
        float sphereSize = 2.1f;

        if (Physics.SphereCast(rayDown, sphereSize, out hitInfo, 5.0f, groundLayer))
        {
            transform.rotation = Quaternion.Slerp(transform.rotation,
                Quaternion.LookRotation(Vector3.Cross(transform.right, hitInfo.normal)),
                1 - Mathf.Exp(-moveSpeed * Time.deltaTime));
        }
        // Regular grounding. Grounds 0.5m above the ground when detecting something in a sphere below itself.
        if (Physics.SphereCast(rayDown, sphereSize, out hitInfo, 3.0f, groundLayer))
        {
            //if (!rotating)
                //transform.rotation = Quaternion.Slerp(transform.rotation, transform.rotation, 1 - Mathf.Exp(-moveSpeed * Time.deltaTime));
            //transform.rotation = Quaternion.LookRotation(Vector3.Cross(transform.right, hitInfo.normal));
            transform.position = hitInfo.point + hitInfo.normal * 2.0f;
            grounded = true;
        }
        else
        {
            grounded = false;
        }
        return;
    }
    #endregion
    
    /// <summary>
    /// If not grounded, the unit should fall
    /// </summary>
    void ApplyGravity()
    {
        if (!grounded)
        {
            transform.position += Physics.gravity * Time.deltaTime;
        }
    }
}

