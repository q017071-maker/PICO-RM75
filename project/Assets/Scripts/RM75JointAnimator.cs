using System;
using UnityEngine;

public sealed class RM75JointAnimator : MonoBehaviour
{
    [Header("URDF joint order: joint_1 through joint_7")]
    [SerializeField] Transform[] jointTransforms = new Transform[7];
    [SerializeField] Vector3[] jointAxes = new Vector3[7];
    [SerializeField] Renderer[] modelRenderers = new Renderer[0];
    [SerializeField] float interpolationTime = 0.12f;
    [SerializeField] double staleAfterSeconds = 0.60;
    [SerializeField] Color normalColor = new Color(0.75f, 0.75f, 0.75f, 1f);
    [SerializeField] Color staleColor = new Color(0.32f, 0.35f, 0.38f, 1f);

    static readonly int ColorProperty = Shader.PropertyToID("_Color");
    Quaternion[] zeroRotations;
    float[] currentAngles = new float[7];
    float[] targetAngles = new float[7];
    RM75StateSnapshot latest;
    long latestSequence = -1;
    bool initialized;
    bool staleVisual = true;
    MaterialPropertyBlock propertyBlock;

    void Awake()
    {
        if (jointTransforms == null || jointTransforms.Length != 7 ||
            jointAxes == null || jointAxes.Length != 7)
        {
            Debug.LogError("[RM75_VIEW] Animator requires seven joint transforms and axes");
            return;
        }
        zeroRotations = new Quaternion[7];
        for (int index = 0; index < 7; index++)
        {
            if (jointTransforms[index] == null || jointAxes[index].sqrMagnitude < 1e-8f)
            {
                Debug.LogError("[RM75_VIEW] Animator has an invalid joint reference at " + index);
                return;
            }
            jointAxes[index] = jointAxes[index].normalized;
            zeroRotations[index] = jointTransforms[index].localRotation;
        }
        if (modelRenderers == null || modelRenderers.Length == 0)
            modelRenderers = GetComponentsInChildren<Renderer>(true);
        propertyBlock = new MaterialPropertyBlock();
        initialized = true;
        SetVisualColor(staleColor);
    }

    public void SetSnapshot(RM75StateSnapshot snapshot)
    {
        if (snapshot.JointDegrees == null || snapshot.JointDegrees.Length != 7 ||
            snapshot.Sequence <= latestSequence)
            return;
        latest = snapshot;
        latestSequence = snapshot.Sequence;
        Array.Copy(snapshot.JointDegrees, targetAngles, 7);
    }

    void Update()
    {
        if (!initialized)
            return;
        float safeTime = Math.Max(0.001f, interpolationTime);
        float blend = 1f - Mathf.Exp(-Time.deltaTime / safeTime);
        for (int index = 0; index < 7; index++)
        {
            currentAngles[index] += ShortestDeltaDegrees(
                currentAngles[index], targetAngles[index]) * blend;
            jointTransforms[index].localRotation = zeroRotations[index] *
                Quaternion.AngleAxis(currentAngles[index], jointAxes[index]);
        }
        bool stale = latestSequence < 0 || latest.IsStale(
            RM75StateReceiver.MonotonicSeconds, staleAfterSeconds);
        if (stale != staleVisual)
        {
            staleVisual = stale;
            SetVisualColor(stale ? staleColor : normalColor);
        }
    }

    void SetVisualColor(Color color)
    {
        if (propertyBlock == null)
            propertyBlock = new MaterialPropertyBlock();
        propertyBlock.SetColor(ColorProperty, color);
        if (modelRenderers == null)
            return;
        foreach (Renderer item in modelRenderers)
            if (item != null)
                item.SetPropertyBlock(propertyBlock);
    }

    public static float ShortestDeltaDegrees(float from, float to)
    {
        return Mathf.Repeat(to - from + 180f, 360f) - 180f;
    }
}
