using System.Collections;
using UnityEngine;

public class LegStepTest : MonoBehaviour
{
    [SerializeField] Transform homeTransform;
    [SerializeField] float wantStepAtDistance = 2f;
    [SerializeField] float moveDuration = 0.2f;
    [SerializeField] float stepOvershootFraction = 0.2f;
    public bool Moving;
    public bool debug;

    RaycastHit hitInfo;
    bool grounded;
    Vector3 forward;
    float groundAngle;

    public void TryMove()
    {
        if (Moving) return;
        float distFromHome = Vector3.Distance(transform.position, homeTransform.position);

        if (distFromHome > wantStepAtDistance)
            StartCoroutine(Move());
    }

    IEnumerator Move()
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
}
