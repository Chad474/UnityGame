using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class LegControllerTest : MonoBehaviour
{
    #region Variables
    [SerializeField] Transform target;
    [SerializeField] LegStepTest legStepTest;
    [SerializeField] Transform homeTransform;
    [SerializeField] float turnSpeed;
    [SerializeField] float moveSpeed;
    [SerializeField] float turnAcceleration;
    [SerializeField] float moveAcceleration;
    [SerializeField] float maxAngToTarget;
    Vector3 currentVelocity;
    Vector3 currentHomeVelocity;
    float maxDistToTarget = 4f;
    // We are only doing a rotation around the up axis, so we only use a float here
    float currentAngularVelocity;

    public LayerMask groundLayer;
    public bool debug;
    public bool grounded, groundedHome, rotating, moving;
    public float angleToTarget;
    float legLength = 3.0f;
    #endregion

    void LateUpdate()
    {
        
        Ground();
        CheckAhead();
        //GroundHomePosition();
        ApplyGravity();
        RootMotionUpdate();    
    }

    private void Update()
    {
        if(grounded && groundedHome)
            StartCoroutine(LegUpdateCoroutine());
    }
    IEnumerator LegUpdateCoroutine()
    {
        while (true)
        {
            legStepTest.TryMove();
            yield return null;
        }
    }

    #region Root Motion
    void RootMotionUpdate()
    {
        // Get the direction toward our target
        Vector3 towardTarget = target.position - transform.position;
        // Vector toward target on the local XZ plane
        Vector3 towardTargetProjected = Vector3.ProjectOnPlane(towardTarget, transform.up);
        // Get the angle from the forward direction to the direction toward our target
        // Here we get the signed angle around the up vector so we know which direction to turn in
        float angToTarget = Vector3.SignedAngle(transform.forward, towardTargetProjected, transform.up);
        angleToTarget = angToTarget;
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
        currentAngularVelocity = Mathf.Lerp(currentAngularVelocity, targetAngularVelocity, 1 - Mathf.Exp(-turnAcceleration * Time.deltaTime));

        // Rotate the transform around the Y axis in world space, 
        // making sure to multiply by delta time to get a consistent angular velocity
        transform.Rotate(0, Time.deltaTime * currentAngularVelocity, 0, Space.Self);

 
        // Movement
        Vector3 targetVelocity = Vector3.zero;

        // Don't move if we're facing away from the target, just rotate in place
        if (Mathf.Abs(angToTarget) < maxAngToTarget)
        {
            rotating = false;
            float distToTarget = Vector3.Distance(transform.position, target.position);

            // If we're too far away, approach the target
            if (distToTarget > maxDistToTarget)
            {
                targetVelocity = moveSpeed * transform.forward; //towardTarget.normalized;
                
            }
        }
        else
        {
            
            rotating = true;

            
            float ang = Vector3.Angle(transform.forward, homeTransform.forward);
            // Arbitrary angle to be between.
            if (ang > maxAngToTarget/2 || ang < -maxAngToTarget/2)
            {
                // Rotate the home position to be the same as the legs rotation after the leg turns.
                Vector3 diff = transform.up - homeTransform.up;
                if (Mathf.Abs(diff.x) < 0.25 && Mathf.Abs(diff.y) < 0.25 && Mathf.Abs(diff.z) < 0.25)
                    homeTransform.forward = transform.forward;
            }

        }
        currentVelocity = Vector3.Lerp(currentVelocity, targetVelocity, 1 - Mathf.Exp(-moveAcceleration * Time.deltaTime));

        transform.position += currentVelocity * Time.deltaTime;
    }
    #endregion

    #region Grounding
  
    void Ground()
    {
        var rayDown = new Ray(transform.position + (transform.up * 0.5f), -transform.up);
        //var rayDiagonalNegative = new Ray(transform.position + (transform.up * 0.05f), transform.TransformDirection(0,-1,-1));
        var hitInfo = new RaycastHit();
        float sphereSize = 0.5f;
        
        // Regular grounding. Grounds 0.5m above the ground when detecting something in a sphere below itself.
        if (Physics.SphereCast(rayDown, sphereSize, out hitInfo, 2.0f, groundLayer))
        {
            if(!rotating)
                transform.rotation = Quaternion.Slerp(transform.rotation, homeTransform.rotation, 1 - Mathf.Exp(-moveSpeed * Time.deltaTime));
            transform.position = hitInfo.point + hitInfo.normal * 0.5f;
            grounded = true;
        }
        else
        {
            grounded = false;
        }
        return;
    }

    public void GroundHomePosition()
    {
        // Ground the home position with a temp variable first
        Vector3 newHomePos = homeTransform.position;
        Quaternion newHomeRot = homeTransform.rotation;
        GroundHome(ref newHomePos, ref newHomeRot);
        
        // Rotate the home position to be correct with whatever it runs into.
        homeTransform.rotation = newHomeRot;

        // Add velocity.
        Vector3 targetHomeVelocity = Vector3.zero;
        targetHomeVelocity = moveSpeed * homeTransform.forward;
        currentHomeVelocity = Vector3.Lerp(currentHomeVelocity, targetHomeVelocity, 1 - Mathf.Exp(-moveAcceleration * Time.deltaTime));
        newHomePos += currentHomeVelocity * Time.deltaTime;
        Vector3 headingPos = newHomePos - transform.position;
        // Normalize it to clamp to a radius of 3.
        Vector3 normalizedOffset = headingPos.normalized;
        float offsetLength = headingPos.magnitude;
        float clampedRadius = Mathf.Clamp(offsetLength, 2.5f, 3.5f);
        // Position the home and clamp it so it's never more than 3 units away from the leg.
        homeTransform.position = transform.position + (normalizedOffset * clampedRadius);


    }

    void GroundHome(ref Vector3 pos, ref Quaternion rot)
    {
        var rayDown = new Ray(pos + (homeTransform.up * 0.5f), -homeTransform.up);
        var rayForward = new Ray(pos + (homeTransform.up * 0.5f), homeTransform.forward);
        //var rayDiagonalNegative = new Ray(pos + (homeTransform.up * 0.5f), homeTransform.TransformDirection(0, -1, -1));
        var hitInfo = new RaycastHit();
        float sphereSize = 0.05f;
        float sphereDistance = 2.0f;

        // Forward raycast to look for a wall.
        if (Physics.SphereCast(rayForward, sphereSize, out hitInfo, 0.5f, groundLayer))
        {
            float ang = Vector3.Angle(hitInfo.normal, homeTransform.up);
            if (ang < 91)
            {
                rot = Quaternion.LookRotation(Vector3.Cross(homeTransform.right, hitInfo.normal));
                pos = hitInfo.point + hitInfo.normal * 0.5f;
                groundedHome = true;
            }
        }
        // Raycast to look down in order to ground the home position and rotate to whatever it is grounded to.
        else if (Physics.SphereCast(rayDown, 0.6f, out hitInfo, sphereDistance, groundLayer))
        {
            rot = Quaternion.LookRotation(Vector3.Cross(homeTransform.right, hitInfo.normal));
            pos = hitInfo.point + hitInfo.normal * 0.5f;
            groundedHome = true;
        }
        else
            groundedHome = false;
        return;
    }

    #endregion

    /// <summary>
    /// Check ahead for a spot to ground the home position in front of the leg
    /// </summary>
    void CheckAhead()
    {
        Vector3 groundPosition = homeTransform.position;
        Quaternion groundRotation = homeTransform.rotation;
        float sphereRadius = 0.6f;
        float rayDistance = 1.5f;
        // Buffer for our nonalloc physics checks
        Collider[] collisionResults = new Collider[20];

        // Check ahead of our leg by 2 in the forward direction
        int hitCount = Physics.OverlapSphereNonAlloc(transform.position + transform.forward * legLength,
            sphereRadius, collisionResults, groundLayer);

        // If we only hit the ground, simply ground home position.
        if (hitCount == 1)
        {
            Vector3 origin = transform.position + (transform.forward * legLength);
            // Use a raycast that checks below the homeposition to ground itself.
            GroundByRaycast(origin, -homeTransform.up, rayDistance, ref groundPosition, ref groundRotation);
        }
        // If we hit anything else, check if we can ground to it.
        else if (hitCount > 1)
        {
            // TODO make logic to solve all possible points to jump to if more than 2 possible points are present

            var rayForward = new Ray(homeTransform.position + (homeTransform.up * 0.5f), homeTransform.forward);
            // Ground the home position using a spherecast that checks forward.
            GroundBySphereCast(rayForward, sphereRadius, rayDistance, ref groundPosition, ref groundRotation);
        }
        // If we hit nothing, we reached an edge. Check if we can ground over the edge.
        else if (hitCount == 0)
        {
            
            CheckLedge(out groundPosition, out groundRotation);
        }

        homeTransform.position = groundPosition;
        homeTransform.rotation = groundRotation;
    }

    void CheckLedge(out Vector3 ledgePoint, out Quaternion ledgeNormal)
    {
        ledgePoint = homeTransform.position;
        ledgeNormal = homeTransform.rotation;


        float horizontalDistanceCheck = 2.0f;
        float verticalDistanceCheck = 2.0f;


        // Horizontal rays to check above and over the ledge
        Vector3 origin = transform.position + transform.forward * legLength;
        if (Physics.Raycast(origin, -transform.up, horizontalDistanceCheck, groundLayer))
        {
            // Exit if it's not clear
            //Debug.DrawRay(origin, -transform.up * horizontalDistanceCheck, Color.red);
        }
        else
        {
            // if no hit, cast down from the ends of our previous rays
            // Move origin to the end of the previous ray
            origin += -transform.up * horizontalDistanceCheck;
            GroundByRaycast(origin, -transform.forward, verticalDistanceCheck, ref ledgePoint, ref ledgeNormal);
        }
        // Capsule cast to find the ledge point and normal.
        /*
        float capsuleHeight = 1.0f;
        float capsuleRadius = 0.5f;
        float capsuleCastCheckDistance = 1f;
        Vector3 capsuleTop = transform.position + (capsuleHeight + capsuleRadius * 2) * Vector3.up;
        // 1 meter behind the top sphere on the character capsule
        Vector3 capsuleBottom = capsuleTop - Vector3.back * capsuleRadius * 2;
        capsuleBottom -= -transform.up * capsuleRadius * 2;
        // 45 degrees from down
        Vector3 dir = (Vector3.back - transform.up) / 2;
        RaycastHit capsuleHit;
        if (Physics.CapsuleCast(capsuleTop, capsuleBottom, capsuleRadius, dir, out capsuleHit,
            capsuleCastCheckDistance, groundLayer, QueryTriggerInteraction.Ignore))
        {
            ledgePoint = capsuleHit.point + homeTransform.up * 0.5f;
            ledgeNormal = Quaternion.LookRotation(Vector3.Cross(homeTransform.right, capsuleHit.normal));
            groundedHome = true;
        }
        else
        {
            groundedHome = false;
            Debug.Log("Capsule hit failed.");
        }
        */
        //return false;
    }

    /// <summary>
    /// Ground and rotate using Physics.Raycast
    /// </summary>
    /// <param name="origin"></param>
    /// <param name="maxDistance"></param>
    /// <param name="groundPosition"></param>
    /// <param name="groundRotation"></param>
    void GroundByRaycast(Vector3 origin, Vector3 direction, float maxDistance, ref Vector3 groundPosition, ref Quaternion groundRotation)
    {
        var hitInfo = new RaycastHit();
        if (Physics.Raycast(origin, direction, out hitInfo, maxDistance, groundLayer))
        {
            groundRotation = Quaternion.LookRotation(Vector3.Cross(homeTransform.right, hitInfo.normal));
            groundPosition = hitInfo.point + hitInfo.normal * 0.5f;
            groundedHome = true;
        } else
        {
            groundedHome = false;
        }
    }

    /// <summary>
    /// Ground and rotate using Physics.SphereCast
    /// </summary>
    /// <param name="ray"></param>
    /// <param name="radius"></param>
    /// <param name="maxDistance"></param>
    /// <param name="groundPosition"></param>
    /// <param name="groundRotation"></param>
    void GroundBySphereCast(Ray ray, float radius, float maxDistance, ref Vector3 groundPosition, ref Quaternion groundRotation)
    {
        var hitInfo = new RaycastHit();
        if (Physics.SphereCast(ray, radius, out hitInfo, maxDistance, groundLayer))
        {
            groundRotation = Quaternion.LookRotation(Vector3.Cross(homeTransform.right, hitInfo.normal));
            groundPosition = hitInfo.point + hitInfo.normal * 0.5f;
            groundedHome = true;

        } else
        {
            groundedHome = false;
        }
    }

    /// <summary>
    /// If not grounded, the unit should fall
    /// </summary>
    void ApplyGravity()
    {
        if (!grounded)
        {
            transform.position += Physics.gravity * Time.deltaTime;
            StopCoroutine(LegUpdateCoroutine());
        }
        if (!groundedHome)
            homeTransform.position += Physics.gravity * Time.deltaTime;
    }

    /// <summary>
    /// Draw debug lines for the grounded check
    /// ...the height
    /// ...and the height padding
    /// </summary>
    void DrawDebugLines()
    {
        if (!debug) return;
        float height = 2.0f;
        Debug.DrawLine(transform.position, transform.position + transform.forward * height * 2, Color.blue);
        Debug.DrawLine(transform.position, transform.position + Vector3.down * height, Color.green);
        Debug.DrawLine(transform.position, transform.position + transform.TransformDirection(0, -1, -1) * height, Color.yellow);

        Debug.DrawLine(homeTransform.position, homeTransform.position + homeTransform.forward * 0.5f, Color.blue);
        Debug.DrawLine(homeTransform.position, homeTransform.position - homeTransform.up * height, Color.green);
        //Debug.DrawLine(homeTransform.position, homeTransform.position + homeTransform.TransformDirection(0, -1, -1) * height/2, Color.yellow);
        //Debug.DrawLine(homeTransform.position, homeTransform.position + homeTransform.TransformDirection(0, -1, 1) * height/2, Color.cyan);
    }


}
