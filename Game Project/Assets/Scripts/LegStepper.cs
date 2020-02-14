using System.Collections;
using UnityEngine;

public class LegStepper : MonoBehaviour
{
    // The position and rotation we want to stay in range of
    [SerializeField] Transform homeTransform;
    // Stay within this distance of home
    [SerializeField] float wantStepAtDistance;
    // How long a step takes to complete
    [SerializeField] float moveDuration;
    // Fraction of the max distance from home we want to overshoot by
    [SerializeField] float stepOvershootFraction;
    [SerializeField] LayerMask groundLayer;
    [SerializeField] Transform pillar;
    private Transform spiderCore;
    private float attachMoveDuration = 0.8f;
    // Is the leg moving?
    public bool Moving;

    private float legHeight = 0.5f;
    public float height = 2.0f;
    public float heightPadding = 0.15f;
    public float maxGroundAngle = 360;
    public bool debug;
    Vector3 forward;
    RaycastHit hitInfo;
    bool grounded;
    float angle;
    float groundAngle;

    // Movement functions, should a function of the parent but I'm lazy.
    public float moveSpeed = 10;
    private Vector3 currentHomeVelocity;



    #region Move individual leg
    public void TryMoveToHome()
    {
        // If we are already moving, don't start another move
        if (Moving) return;

        // Only move if we are not close enough to our target.
        float distFromHome = Vector3.Distance(transform.position, homeTransform.position);
        if (distFromHome > wantStepAtDistance)
        {
            // Start the step coroutine
            StartCoroutine(MoveToHome());
        }
    }
    IEnumerator MoveToHome()
    {
        Moving = true;
        Quaternion startRot = transform.rotation;
        Vector3 startPoint = transform.position;
        Quaternion endRot = homeTransform.rotation;
        // Directional vector from the foot to the home position
        Vector3 towardHome = homeTransform.position - transform.position;
        float overshootDistance = wantStepAtDistance * stepOvershootFraction;
        Vector3 overshootVector = towardHome * overshootDistance;

        overshootVector = Vector3.Project(overshootVector, homeTransform.up);
        Vector3 endPoint = homeTransform.position + overshootVector;

        Vector3 centerPoint = (startPoint + endPoint) / 2;
        // But also lift off, so we move it up by half the step distance (arbitrarily)
        //centerPoint += homeTransform.up * Vector3.Distance(startPoint, endPoint) / 2f;
        float timeElapsed = 0;
        do
        {
            timeElapsed += Time.deltaTime;
            float normalizedTime = timeElapsed / moveDuration;
            normalizedTime = Easing.Cubic.InOut(normalizedTime);
            //Quadratic bezier curve
            transform.position = Vector3.Lerp(
                                    Vector3.Lerp(startPoint, centerPoint, normalizedTime),
                                    Vector3.Lerp(centerPoint, endPoint, normalizedTime),
                                    normalizedTime);

            transform.rotation = Quaternion.Slerp(startRot, endRot, normalizedTime);
            yield return null;
        } while (timeElapsed < moveDuration);
        Moving = false;
    }
    #endregion

    #region Move legs
    public void TryMove()
    {
        
        // If we are already moving, don't start another move
        if (Moving) return;

        float distFromHome = Vector3.Distance(transform.position, homeTransform.position);
        /*
        // Lift the home position up, then ground it.
        Vector3 newHomePos = homeTransform.position;
        Quaternion newHomeRot = homeTransform.rotation;
        newHomePos = Ground(newHomePos);
        homeTransform.position = newHomePos;
        homeTransform.rotation = newHomeRot;

        // Normalize it to clamp to a radius of 3.
        Vector3 headingPos = homeTransform.position - transform.position;
        Vector3 normalizedOffset = headingPos.normalized;
        float offsetLength = headingPos.magnitude;
        float clampedRadius = Mathf.Clamp(offsetLength, 2.5f, 3.5f);
        // Position the home and clamp it so it's never more than 3 units away from the leg.
        homeTransform.position = transform.position + (normalizedOffset * clampedRadius);
        */

        GroundHomePosition();
        // If we are too far off in position or rotation
        if (distFromHome > wantStepAtDistance)
        {
            // Start the step coroutine
            StartCoroutine(Move());
        }
    }

    // Coroutines must return an IEnumerator
    IEnumerator Move()
    {
        // Indicate we're moving (used later)
        Moving = true;

        // Store the initial conditions
        Quaternion startRot = transform.rotation;
        Vector3 startPoint = transform.position;

        Quaternion endRot = homeTransform.rotation;

        // Directional vector from the foot to the home position
        Vector3 towardHome = homeTransform.position - transform.position;
        // Total distance to overshoot by
        float overshootDistance = wantStepAtDistance * stepOvershootFraction;
        Vector3 overshootVector = towardHome * overshootDistance;
        // Since we don't ground the point in this simplified implementation,
        // we restrict the overshoot vector to be level with the ground
        // by projecting it on the world XZ plane.
        //overshootVector = Vector3.ProjectOnPlane(overshootVector, Vector3.up);

        // Ground the foot
        //(homeTransform, overshootVector) = Ground(homeTransform, overshootVector);


        Vector3 endPoint = homeTransform.position + overshootVector;
        // We want to pass through the center point
        Vector3 centerPoint = (startPoint + endPoint) / 2;
        // But also lift off, so we move it up by half the step distance (arbitrarily)
        centerPoint += homeTransform.up * Vector3.Distance(startPoint, endPoint) / 2f;
        float timeElapsed = 0;
        do
        {
            timeElapsed += Time.deltaTime;
            float normalizedTime = timeElapsed / moveDuration;
            normalizedTime = Easing.Cubic.InOut(normalizedTime);

            //Quadratic bezier curve
            transform.position = Vector3.Lerp(
                                    Vector3.Lerp(startPoint, centerPoint, normalizedTime),
                                    Vector3.Lerp(centerPoint, endPoint, normalizedTime),
                                    normalizedTime);

            transform.rotation = Quaternion.Slerp(startRot, endRot, normalizedTime);
            yield return null;
            
        } while (timeElapsed < moveDuration);

        // Done moving
        Moving = false;

        // Reset home transform position.
        homeTransform.position = pillar.position;
    }
    #endregion

    #region Attach legs
    public bool TryAttach(Transform startTarget, Transform endTarget, GameObject s, Transform spiderLegTarget)
    {
        if (Moving) return false;
        StartCoroutine(Attach(startTarget, endTarget, s, spiderLegTarget));
        return true;
    }

    IEnumerator Attach(Transform startTarget, Transform endTarget, GameObject spiderCore, Transform spiderLegTarget)
    {
        // Indicate we're moving
        Moving = true;

        // Store the initial conditions
        Quaternion startRot = startTarget.rotation;
        Vector3 startPoint = startTarget.position;
        Quaternion endRot = endTarget.rotation;
        Vector3 endPoint = endTarget.position;

        // We want to pass through the center point
        Vector3 centerPoint = startPoint;
        // But also lift off, so we move it up by half the distance
        //centerPoint += startTarget.up * Vector3.Distance(startPoint, endPoint) / 2f;
        centerPoint += startTarget.forward * Vector3.Distance(startPoint, endPoint) / 2f;
        centerPoint += startTarget.right * Vector3.Distance(startPoint, endPoint) / 2f;

        float timeElapsed = 0;
        do
        {
            timeElapsed += Time.deltaTime;
            float normalizedTime = timeElapsed / attachMoveDuration;
            normalizedTime = Easing.Cubic.InOut(normalizedTime);

            //Quadratic bezier curve
            startTarget.position = Vector3.Lerp(
                                    Vector3.Lerp(startPoint, centerPoint, normalizedTime),
                                    Vector3.Lerp(centerPoint, endPoint, normalizedTime),
                                    normalizedTime);
            startTarget.rotation = Quaternion.Slerp(startRot, endRot, normalizedTime);

            yield return null;
        } while (timeElapsed < attachMoveDuration);

        // Done moving
        Moving = false;

        // Start a second coroutine to get the leg firmly in place.
        SpiderController s = spiderCore.GetComponent<SpiderController>();
        StartCoroutine(Attach2(spiderLegTarget));
        s.isEnabled++;
    }
    IEnumerator Attach2(Transform endTarget)
    {
        // Store the initial conditions
        Quaternion startRot = transform.rotation;
        Vector3 startPoint = transform.position;
        Quaternion endRot = endTarget.rotation;
        // Ground the foot
        Vector3 endPoint = Ground(endTarget.position);
        float timeElapsed = 0;
        do
        {
            // Add time since last frame to the time elapsed
            timeElapsed += Time.deltaTime;

            float normalizedTime = timeElapsed / moveDuration;
            normalizedTime = Easing.Cubic.InOut(normalizedTime);

            // Interpolate position and rotation
            transform.position = Vector3.Lerp(startPoint, endPoint, normalizedTime);
            transform.rotation = Quaternion.Slerp(startRot, endRot, normalizedTime);

            // Wait for one frame
            yield return null;
        }
        while (timeElapsed < moveDuration);
    }
    #endregion

    #region Ground Home
    public void GroundHomePosition()
    {
        // Ground the home position with a temp variable first
        Vector3 newHomePos = homeTransform.position;
        Quaternion newHomeRot = homeTransform.rotation;
        GroundHome(ref newHomePos, ref newHomeRot);

        // Rotate the home position to be correct with whatever it runs into.
        homeTransform.rotation = newHomeRot;
        
        /*
        // Update position with velocity.
        Vector3 targetHomeVelocity = Vector3.zero;
        targetHomeVelocity = moveSpeed * homeTransform.forward;
        currentHomeVelocity = Vector3.Lerp(currentHomeVelocity, targetHomeVelocity, 1 - Mathf.Exp(-moveSpeed * Time.deltaTime));
        newHomePos += currentHomeVelocity * Time.deltaTime;
        */
        Vector3 headingPos = newHomePos - pillar.position;

        // Normalize it to clamp to a radius of 3.
        Vector3 normalizedOffset = headingPos.normalized;
        float offsetLength = headingPos.magnitude;
        float clampedRadius = Mathf.Clamp(offsetLength, 0.01f, 2.0f);
        // Position the home and clamp it so it's never more than 3 units away from the leg.
        homeTransform.position = pillar.position + (normalizedOffset * clampedRadius);
        //homeTransform.position = newHomePos;

    }

    void GroundHome(ref Vector3 pos, ref Quaternion rot)
    {
        var rayDown = new Ray(pos + (homeTransform.up * 0.50f), -homeTransform.up);
        var rayForward = new Ray(pos + (homeTransform.up * 0.50f), homeTransform.forward);
        //var rayDiagonalNegative = new Ray(pos + (homeTransform.up * 0.5f), homeTransform.TransformDirection(0, -1, -1));
        var hitInfo = new RaycastHit();
        float sphereSize = 0.05f;
        float sphereDistance = 3.0f;

        // Forward raycast to look for a wall.
        if (Physics.SphereCast(rayForward, sphereSize, out hitInfo, 0.5f, groundLayer))
        {
            float ang = Vector3.Angle(hitInfo.normal, homeTransform.up);
            if (ang < 91)
            {
                rot = Quaternion.LookRotation(Vector3.Cross(homeTransform.right, hitInfo.normal));
                pos = hitInfo.point + hitInfo.normal * 0.5f;
                //groundedHome = true;
            }
        }
        else if (Physics.SphereCast(rayDown, 0.6f, out hitInfo, sphereDistance, groundLayer))
        {
            rot = Quaternion.LookRotation(Vector3.Cross(homeTransform.right, hitInfo.normal));
            pos = hitInfo.point + hitInfo.normal * 0.5f;
            //groundedHome = true;
        }
        else
            //groundedHome = false;
        return;
    }
    #endregion
    #region Grounding
    Vector3 Ground(Vector3 target)
    {
        var rayDown = new Ray(target + (transform.up * 0.5f), -transform.up);
        var hitInfo = new RaycastHit();
        float sphereSize = 0.6f;

        if (Physics.SphereCast(rayDown, sphereSize, out hitInfo, 3.0f, groundLayer))
        {
            target = hitInfo.point + hitInfo.normal * 0.5f;
        }

        return target;
    }
    (Transform, Vector3) Ground(Transform t, Vector3 overshoot)
    {
        var rayDown = new Ray(t.position + (t.up * 0.5f), -t.up);
        var hitInfo = new RaycastHit();
        float sphereSize = 0.6f;

        if (Physics.SphereCast(rayDown, sphereSize, out hitInfo, 3.0f, groundLayer))
        {
            //t.rotation = Quaternion.LookRotation(Vector3.Cross(t.right, hitInfo.normal));
            t.position = hitInfo.point + hitInfo.normal * 0.5f;
            overshoot = Vector3.Project(overshoot, hitInfo.point);
        }

        return (t, overshoot);
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

    /// <summary>
    /// Draw debug lines for the grounded check
    /// ...the height
    /// ...and the height padding
    /// </summary>
    void DrawDebugLines()
    {
        if (!debug) return;

        Debug.DrawLine(transform.position, transform.position + forward * height * 2, Color.blue);
        Debug.DrawLine(transform.position, transform.position - Vector3.up * height, Color.green);
    }

    public void SetHomeTransformPosition(Transform t)
    {
        homeTransform.position = t.position;
    }
    public void SetGroundLayer(LayerMask ground)
    {
        groundLayer = ground;
    }

}
