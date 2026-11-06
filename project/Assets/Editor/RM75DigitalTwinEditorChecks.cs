#if UNITY_EDITOR
using System;
using System.Linq;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

public static class RM75DigitalTwinEditorChecks
{
    const string PrefabPath = "Assets/ThirdParty/RealMan/RM75_6FB/Prefabs/RM75_DigitalTwin.prefab";
    const string ScenePath = "Assets/Scenes/SampleScene.unity";

    public static void Run()
    {
        RM75DigitalTwinBuilder.BuildAssetsAndInstallScene();
        ValidatePrefab();
        ValidateScene();
        Console.WriteLine("RM75 digital twin editor checks passed");
    }

    static void ValidatePrefab()
    {
        GameObject prefab = AssetDatabase.LoadAssetAtPath<GameObject>(PrefabPath);
        Require(prefab != null, "Generated RM75 prefab is missing");
        Require(prefab.name == "RM75_DigitalTwin", "Unexpected prefab root name");
        Require(prefab.GetComponent<RM75FixedPlacement>() != null,
                "RM75FixedPlacement is missing");
        RM75StateReceiver receiver = prefab.GetComponent<RM75StateReceiver>();
        RM75JointAnimator animator = prefab.GetComponent<RM75JointAnimator>();
        Require(receiver != null, "RM75StateReceiver is missing");
        Require(animator != null, "RM75JointAnimator is missing");

        Transform placement = prefab.transform.Find("PlacementRoot");
        Require(placement != null, "PlacementRoot is missing");
        Transform coordinates = placement.Find("CoordinateRoot");
        Require(coordinates != null, "CoordinateRoot is missing");
        Require(coordinates.Find("base_link") != null, "base_link is missing");

        for (int index = 1; index <= 7; index++)
        {
            string jointName = "joint_" + index;
            Require(prefab.GetComponentsInChildren<Transform>(true)
                         .Count(item => item.name == jointName) == 1,
                    jointName + " must occur exactly once");
        }

        MeshFilter[] filters = prefab.GetComponentsInChildren<MeshFilter>(true);
        Renderer[] renderers = prefab.GetComponentsInChildren<Renderer>(true);
        Require(filters.Length == 8, "Expected eight visual meshes");
        Require(renderers.Length == 8, "Expected eight renderers");
        Require(filters.All(item => item.sharedMesh != null),
                "Every MeshFilter must reference a mesh");
        Require(filters.All(item => !item.sharedMesh.isReadable),
                "Imported model meshes must be read-only at runtime");
        Require(prefab.GetComponentsInChildren<Collider>(true).Length == 0,
                "Visualization prefab must not contain colliders");
        Require(prefab.GetComponentsInChildren<Rigidbody>(true).Length == 0,
                "Visualization prefab must not contain rigid bodies");

        Bounds combined = renderers[0].bounds;
        foreach (Renderer item in renderers.Skip(1))
            combined.Encapsulate(item.bounds);
        Require(combined.size.magnitude > 0.1f && combined.size.magnitude < 3f,
                "Robot mesh scale is outside a plausible meter range");

        SerializedObject animatorData = new SerializedObject(animator);
        SerializedProperty joints = animatorData.FindProperty("jointTransforms");
        SerializedProperty axes = animatorData.FindProperty("jointAxes");
        SerializedProperty assignedRenderers = animatorData.FindProperty("modelRenderers");
        Require(joints != null && joints.arraySize == 7,
                "Animator must serialize seven joint transforms");
        Require(axes != null && axes.arraySize == 7,
                "Animator must serialize seven joint axes");
        Require(assignedRenderers != null && assignedRenderers.arraySize == 8,
                "Animator must serialize eight renderers");
        for (int index = 0; index < 7; index++)
        {
            Require(joints.GetArrayElementAtIndex(index).objectReferenceValue != null,
                    "Animator joint reference " + index + " is missing");
            Require(axes.GetArrayElementAtIndex(index).vector3Value.sqrMagnitude > 0.99f,
                    "Animator joint axis " + index + " is not normalized");
            Require(Vector3.Distance(axes.GetArrayElementAtIndex(index).vector3Value,
                                     Vector3.up) < 1e-5f,
                    "ROS Z joint axis must map to Unity Y");
        }

        Vector3[] expectedOrigins =
        {
            new Vector3(0f, 0.2405f, 0f), Vector3.zero,
            new Vector3(0f, 0f, -0.256f), Vector3.zero,
            new Vector3(0f, 0f, -0.21f), Vector3.zero,
            new Vector3(0f, 0f, -0.1612f)
        };
        Transform[] allTransforms = prefab.GetComponentsInChildren<Transform>(true);
        for (int index = 0; index < 7; index++)
        {
            Transform joint = allTransforms.Single(item => item.name == "joint_" + (index + 1));
            Require(Vector3.Distance(joint.localPosition, expectedOrigins[index]) < 1e-5f,
                    "Converted origin is wrong for " + joint.name);
        }

        SerializedObject receiverData = new SerializedObject(receiver);
        Require(receiverData.FindProperty("listenPort").intValue == 5006,
                "Receiver must listen on UDP 5006");
        Require(receiverData.FindProperty("animator").objectReferenceValue == animator,
                "Receiver must reference the prefab animator");

        SerializedObject placementData = new SerializedObject(prefab.GetComponent<RM75FixedPlacement>());
        Require(Mathf.Abs(placementData.FindProperty("distance").floatValue - 1.2f) < 1e-5f,
                "Placement distance must be 1.2 m");
        Require(Mathf.Abs(placementData.FindProperty("verticalOffset").floatValue + 0.75f) < 1e-5f,
                "Placement vertical offset must be -0.75 m");
    }

    static void ValidateScene()
    {
        Scene scene = EditorSceneManager.OpenScene(ScenePath, OpenSceneMode.Single);
        GameObject[] roots = scene.GetRootGameObjects();
        Require(roots.Count(item => item.name == "RM75_DigitalTwin") == 1,
                "SampleScene must contain exactly one RM75_DigitalTwin root");
        GameObject instance = roots.Single(item => item.name == "RM75_DigitalTwin");
        Require(PrefabUtility.GetCorrespondingObjectFromSource(instance) != null,
                "SampleScene robot must be a prefab instance");
    }

    static void Require(bool condition, string message)
    {
        if (!condition)
            throw new InvalidOperationException(message);
    }
}
#endif
