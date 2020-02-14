using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class LegController : MonoBehaviour
{
    [SerializeField] LayerMask groundLayer;
    [SerializeField] GameObject spiderCore;
    [SerializeField] Transform spiderLegTarget;
    [SerializeField] Transform thigh;
    // The target we are going to track
    [SerializeField] Transform target;
    [SerializeField] Transform homeTransform;

    [SerializeField] LegStepper legStepper;

    // How fast we can turn and move full throttle
    [SerializeField] float turnSpeed;
    [SerializeField] float moveSpeed;
    // How fast we will reach the above speeds
    [SerializeField] float turnAcceleration;
    [SerializeField] float moveAcceleration;
    // Try to stay in this range from the target
    float maxDistToTarget = 4f;
    // If we are above this angle from the target, start turning
    [SerializeField] float maxAngToTarget;
    // World space velocity
    Vector3 currentVelocity;
    // We are only doing a rotation around the up axis, so we only use a float here
    private float currentAngularVelocity;

    private Vector3 currentHomeVelocity;
    // We will put all our animation code in LateUpdate.
    // This allows other systems to update the environment first, 
    // allowing the animation system to adapt to it before the frame is drawn.
    [SerializeField] bool isEnabled = true;
    public bool disableUpdate = false;

    private bool grounded, groundedHome, rotating;

    public void Enable()
    {
        isEnabled = true;
    }
    private void Start()
    {
        SpiderController s = spiderCore.GetComponent<SpiderController>();
        s.enabled = false;
    }
    private void Update()
    {
        if (!disableUpdate)
        {
            SpiderController s = spiderCore.GetComponent<SpiderController>();

            if ((s.isEnabled == 4) && !legStepper.Moving && s.enabled == false)
            {
                s.enabled = true;
                s.startCo();
                disableUpdate = true;
                this.enabled = false;
            } else if ((s.isEnabled == 4) && !legStepper.Moving) 
            { 
                disableUpdate = true;
                this.enabled = false;
            }
        }
    }
    void LateUpdate()
    {
        if (!isEnabled) return; // Stops the script.

        if (grounded && groundedHome)
            StartCoroutine(LegUpdateCoroutine());

        Ground();
        GroundHomePosition();
        ApplyGravity();
        RootMotionUpdate();
    }

    IEnumerator LegUpdateCoroutine()
    {
        // Run forever
        while (isEnabled)
        {
            legStepper.TryMoveToHome();
            //legStepper.TryMove();
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
            else
            {
                // Attach to the target
                Attach();
            }
        }
        else
        {
            rotating = true;
            float ang = Vector3.Angle(transform.forward, homeTransform.forward);
            if (ang > 5 || ang < -5)
            {
                homeTransform.rotation = transform.rotation;
                // Normalize the home position and to clamp to a radius of 3.
                Vector3 normalizedOffset = towardTargetProjected.normalized;
                float offsetLength = towardTargetProjected.magnitude;
                float clampedRadius = Mathf.Clamp(offsetLength, 2.5f, 3.5f);
                // Position the home and clamp it so it's never more than 3 units away from the leg.
                homeTransform.position = transform.position + (normalizedOffset * clampedRadius);
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
            if (!rotating)
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
    /// Attaches a leg to a target
    /// </summary>
    public void Attach()
    {
        SpiderController s = spiderCore.GetComponent<SpiderController>();
        if (legStepper.TryAttach(thigh, target, spiderCore, spiderLegTarget))
        {
            isEnabled = false;
        }

        

    }

}
