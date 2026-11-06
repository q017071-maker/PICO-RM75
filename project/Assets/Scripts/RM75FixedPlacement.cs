using UnityEngine;

public sealed class RM75FixedPlacement : MonoBehaviour
{
    [SerializeField] Transform cameraOverride;
    [SerializeField] float distance = 1.2f;
    [SerializeField] float verticalOffset = -0.75f;
    [SerializeField] float yawOffset;
    bool placed;

    void OnEnable()
    {
        if (placed)
            return;
        Transform view = cameraOverride;
        if (view == null && Camera.main != null)
            view = Camera.main.transform;
        if (view == null)
        {
            Debug.LogWarning("[RM75_VIEW] No camera available for fixed placement");
            return;
        }
        Place(view);
    }

    public void Place(Transform cameraTransform)
    {
        if (cameraTransform == null)
            return;
        Vector3 horizontal = new Vector3(cameraTransform.forward.x, 0f,
                                         cameraTransform.forward.z);
        if (horizontal.sqrMagnitude < 1e-8f)
            horizontal = Vector3.forward;
        horizontal = horizontal.normalized;
        transform.position = cameraTransform.position + horizontal * distance +
                             Vector3.up * verticalOffset;
        transform.rotation = Quaternion.LookRotation(horizontal, Vector3.up) *
                             Quaternion.Euler(0f, yawOffset, 0f);
        placed = true;
    }

    [ContextMenu("Place RM75 From Current Camera")]
    public void PlaceFromCurrentCamera()
    {
        placed = false;
        Transform view = cameraOverride;
        if (view == null && Camera.main != null)
            view = Camera.main.transform;
        if (view != null)
            Place(view);
    }
}
