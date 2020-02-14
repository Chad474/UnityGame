using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(CharacterController))]
public class MovementController : MonoBehaviour
{
    private float InputX, InputZ, Speed, gravity;

    private Camera cam;
    private CharacterController characterController;

    private Vector3 desiredMoveDirection;

    private Animator anim;

    [SerializeField] float rotationSpeed = 0.3f;
    [SerializeField] float allowRotation = 0.1f;
    [SerializeField] float movementSpeed = 1f;
    [SerializeField] float gravityMultipler;

    // Start is called before the first frame update
    void Start()
    {
        anim = gameObject.GetComponentInChildren<Animator>();
        characterController = GetComponent<CharacterController>();
        cam = Camera.main;
    }

    // Update is called once per frame
    void Update()
    {

        InputX = Input.GetAxis("Horizontal");
        InputZ = Input.GetAxis("Vertical");

        InputDecider();
        MovementManager();
    }


    void InputDecider()
    {
        Speed = new Vector2(InputX, InputZ).sqrMagnitude;
        if (Speed > 0)
        {
            anim.SetInteger("Speed", 1);
        } else
        {
            anim.SetInteger("Speed", 0);
        }
        if (Speed > allowRotation)
        {
            RotationManager();
        }
        else
        {
            desiredMoveDirection = Vector3.zero;
        }

    }



    void RotationManager()
    {
        var forward = cam.transform.forward;
        var right = cam.transform.right;

        forward.y = 0;
        right.y = 0;

        forward.Normalize();
        right.Normalize();

        desiredMoveDirection = forward * InputZ + right * InputX;

        transform.rotation = Quaternion.Slerp(transform.rotation, Quaternion.LookRotation(desiredMoveDirection), rotationSpeed);

    }

    void MovementManager()
    {
        gravity -= 9.8f * Time.deltaTime;
        gravity = gravity * gravityMultipler;

        Vector3 moveDirection = desiredMoveDirection * (movementSpeed * Time.deltaTime);
        moveDirection = new Vector3(moveDirection.x, gravity, moveDirection.z);
        characterController.Move(moveDirection);
        
        if (characterController.isGrounded)
        {
            gravity = 0;
        }
    }



}
