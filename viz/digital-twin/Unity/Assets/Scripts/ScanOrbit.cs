using UnityEngine;

public sealed class ScanOrbit : MonoBehaviour
{
    public Vector3 target;
    public float distance = 3;
    public float minDistance = 0.1f;
    public float maxDistance = 20;
    public float yaw = -35;
    public float pitch = 20;
    private Vector3 initialTarget;
    private float initialDistance;

    private void Start()
    {
        initialTarget = target;
        initialDistance = distance;
    }

    private void LateUpdate()
    {
        if (Input.GetMouseButton(0))
        {
            yaw += Input.GetAxis("Mouse X") * 4;
            pitch = Mathf.Clamp(pitch - Input.GetAxis("Mouse Y") * 4, -85, 85);
        }
        distance = Mathf.Clamp(distance * Mathf.Exp(-Input.mouseScrollDelta.y * 0.12f), minDistance, maxDistance);
        if (Input.GetMouseButton(1))
            target += (-transform.right * Input.GetAxis("Mouse X") - transform.up * Input.GetAxis("Mouse Y")) * distance * 0.015f;
        if (Input.GetKeyDown(KeyCode.Home))
        {
            target = initialTarget;
            distance = initialDistance;
            yaw = -35;
            pitch = 20;
        }
        if (Input.GetKeyDown(KeyCode.Escape)) Application.Quit();
        ApplyPose();
    }

    public void ApplyPose()
    {
        transform.rotation = Quaternion.Euler(pitch, yaw, 0);
        transform.position = target - transform.forward * distance;
    }
}
