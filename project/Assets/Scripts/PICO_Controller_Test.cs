using UnityEngine;

public class PICO_Controller_Test : MonoBehaviour
{
    public Transform rightController;
    public Transform leftController;

    void Update()
    {
        if (rightController != null)
        {
            Debug.Log(
                "Right Controller Position: "
                + rightController.position
                +
                " Rotation: "
                + rightController.rotation
            );
        }

        if (leftController != null)
        {
            Debug.Log(
                "Left Controller Position: "
                + leftController.position
            );
        }
    }
}