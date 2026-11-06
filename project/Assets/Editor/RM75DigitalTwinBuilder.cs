#if UNITY_EDITOR
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Xml.Linq;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

public static class RM75DigitalTwinBuilder
{
    public const string UrdfPath = "Assets/ThirdParty/RealMan/RM75_6FB/Source/RM75-6FB.urdf";
    public const string MeshDirectory = "Assets/ThirdParty/RealMan/RM75_6FB/Source/Meshes";
    public const string MaterialPath = "Assets/ThirdParty/RealMan/RM75_6FB/Materials/RM75_Normal.mat";
    public const string PrefabPath = "Assets/ThirdParty/RealMan/RM75_6FB/Prefabs/RM75_DigitalTwin.prefab";
    public const string ScenePath = "Assets/Scenes/SampleScene.unity";

    sealed class JointDefinition
    {
        public string Name;
        public string Parent;
        public string Child;
        public Vector3 Position;
        public Quaternion Rotation;
        public Vector3 Axis;
    }

    [MenuItem("RM75/Build Digital Twin In SampleScene")]
    public static void BuildAssetsAndInstallScene()
    {
        EnsureAssetFolders();
        XDocument urdf = XDocument.Load(Path.GetFullPath(UrdfPath));
        string[] links = urdf.Root.Elements("link")
            .Select(item => RequiredAttribute(item, "name")).ToArray();
        XElement[] jointElements = urdf.Root.Elements("joint").ToArray();
        if (jointElements.Any(item => (string)item.Attribute("type") != "revolute"))
            throw new InvalidDataException("RM75 visualization supports revolute joints only");
        JointDefinition[] joints = jointElements.Select(ReadJoint).ToArray();
        if (links.Length != 8 || joints.Length != 7)
            throw new InvalidDataException("RM75 URDF must contain eight links and seven revolute joints");

        foreach (string link in links)
        {
            string meshPath = MeshPath(link);
            AssetDatabase.ImportAsset(meshPath,
                ImportAssetOptions.ForceSynchronousImport | ImportAssetOptions.ForceUpdate);
            if (AssetDatabase.LoadAssetAtPath<Mesh>(meshPath) == null)
                throw new InvalidDataException("Could not import RM75 mesh: " + meshPath);
        }

        Material material = CreateOrUpdateMaterial();
        GameObject model = BuildModel(links, joints, material);
        try
        {
            GameObject prefab = PrefabUtility.SaveAsPrefabAsset(model, PrefabPath);
            if (prefab == null)
                throw new InvalidOperationException("Unity did not save the RM75 prefab");
        }
        finally
        {
            UnityEngine.Object.DestroyImmediate(model);
        }

        InstallPrefabInScene();
        AssetDatabase.SaveAssets();
        AssetDatabase.Refresh();
        ValidateInstalledAssets();
        Debug.Log("[RM75_VIEW] Generated prefab and installed it in " + ScenePath);
    }

    public static void ValidateInstalledAssets()
    {
        GameObject prefab = AssetDatabase.LoadAssetAtPath<GameObject>(PrefabPath);
        if (prefab == null)
            throw new InvalidOperationException("RM75 prefab is missing; run RM75/Build Digital Twin In SampleScene");
        if (prefab.GetComponent<RM75FixedPlacement>() == null ||
            prefab.GetComponent<RM75StateReceiver>() == null ||
            prefab.GetComponent<RM75JointAnimator>() == null)
            throw new InvalidOperationException("RM75 prefab is missing a runtime component");
        if (prefab.GetComponentsInChildren<MeshFilter>(true).Length != 8 ||
            prefab.GetComponentsInChildren<Renderer>(true).Length != 8)
            throw new InvalidOperationException("RM75 prefab must contain eight visual links");
        if (prefab.GetComponentsInChildren<Collider>(true).Length != 0 ||
            prefab.GetComponentsInChildren<Rigidbody>(true).Length != 0)
            throw new InvalidOperationException("RM75 visualization prefab must not contain physics components");

        RM75JointAnimator animator = prefab.GetComponent<RM75JointAnimator>();
        RM75StateReceiver receiver = prefab.GetComponent<RM75StateReceiver>();
        var animatorData = new SerializedObject(animator);
        SerializedProperty jointReferences = animatorData.FindProperty("jointTransforms");
        SerializedProperty jointAxes = animatorData.FindProperty("jointAxes");
        if (jointReferences == null || jointReferences.arraySize != 7 ||
            jointAxes == null || jointAxes.arraySize != 7)
            throw new InvalidOperationException("RM75 animator must contain seven joint references and axes");
        for (int index = 0; index < 7; index++)
        {
            if (jointReferences.GetArrayElementAtIndex(index).objectReferenceValue == null ||
                jointAxes.GetArrayElementAtIndex(index).vector3Value.sqrMagnitude < 0.99f)
                throw new InvalidOperationException("RM75 animator joint " + (index + 1) + " is invalid");
        }
        var receiverData = new SerializedObject(receiver);
        if (receiverData.FindProperty("listenPort").intValue != 5006 ||
            receiverData.FindProperty("animator").objectReferenceValue != animator)
            throw new InvalidOperationException("RM75 receiver configuration is invalid");

        Scene scene = EditorSceneManager.OpenScene(ScenePath, OpenSceneMode.Single);
        int instanceCount = scene.GetRootGameObjects()
            .Count(item => item.name == "RM75_DigitalTwin");
        if (instanceCount != 1)
            throw new InvalidOperationException("SampleScene must contain exactly one RM75_DigitalTwin instance");
    }

    static GameObject BuildModel(string[] links, JointDefinition[] joints, Material material)
    {
        var root = new GameObject("RM75_DigitalTwin");
        RM75FixedPlacement placement = root.AddComponent<RM75FixedPlacement>();
        RM75JointAnimator animator = root.AddComponent<RM75JointAnimator>();
        RM75StateReceiver receiver = root.AddComponent<RM75StateReceiver>();

        var placementRoot = new GameObject("PlacementRoot");
        placementRoot.transform.SetParent(root.transform, false);
        var coordinateRoot = new GameObject("CoordinateRoot");
        coordinateRoot.transform.SetParent(placementRoot.transform, false);

        var linkObjects = new Dictionary<string, GameObject>();
        foreach (string link in links)
            linkObjects.Add(link, CreateLink(link, material));
        linkObjects["base_link"].transform.SetParent(coordinateRoot.transform, false);

        var jointObjects = new Transform[joints.Length];
        var jointAxes = new Vector3[joints.Length];
        for (int index = 0; index < joints.Length; index++)
        {
            JointDefinition joint = joints[index];
            if (!linkObjects.ContainsKey(joint.Parent) || !linkObjects.ContainsKey(joint.Child))
                throw new InvalidDataException("Joint references an unknown link: " + joint.Name);
            var pivot = new GameObject(joint.Name);
            pivot.transform.SetParent(linkObjects[joint.Parent].transform, false);
            pivot.transform.localPosition = joint.Position;
            pivot.transform.localRotation = joint.Rotation;
            linkObjects[joint.Child].transform.SetParent(pivot.transform, false);
            jointObjects[index] = pivot.transform;
            jointAxes[index] = joint.Axis.normalized;
        }

        Renderer[] renderers = root.GetComponentsInChildren<Renderer>(true);
        SetObjectArray(animator, "jointTransforms", jointObjects);
        SetVectorArray(animator, "jointAxes", jointAxes);
        SetObjectArray(animator, "modelRenderers", renderers);
        var receiverData = new SerializedObject(receiver);
        receiverData.FindProperty("listenPort").intValue = 5006;
        receiverData.FindProperty("animator").objectReferenceValue = animator;
        receiverData.ApplyModifiedPropertiesWithoutUndo();
        EditorUtility.SetDirty(placement);
        return root;
    }

    static GameObject CreateLink(string name, Material material)
    {
        Mesh mesh = AssetDatabase.LoadAssetAtPath<Mesh>(MeshPath(name));
        if (mesh == null)
            throw new InvalidDataException("Missing imported mesh for " + name);
        var item = new GameObject(name);
        item.AddComponent<MeshFilter>().sharedMesh = mesh;
        item.AddComponent<MeshRenderer>().sharedMaterial = material;
        return item;
    }

    static void InstallPrefabInScene()
    {
        Scene scene = EditorSceneManager.OpenScene(ScenePath, OpenSceneMode.Single);
        foreach (GameObject existing in scene.GetRootGameObjects()
                     .Where(item => item.name == "RM75_DigitalTwin").ToArray())
            UnityEngine.Object.DestroyImmediate(existing);
        GameObject prefab = AssetDatabase.LoadAssetAtPath<GameObject>(PrefabPath);
        GameObject instance = PrefabUtility.InstantiatePrefab(prefab, scene) as GameObject;
        if (instance == null)
            throw new InvalidOperationException("Could not instantiate RM75 prefab in SampleScene");
        instance.name = "RM75_DigitalTwin";
        EditorSceneManager.MarkSceneDirty(scene);
        if (!EditorSceneManager.SaveScene(scene, ScenePath))
            throw new IOException("Could not save " + ScenePath);
    }

    static Material CreateOrUpdateMaterial()
    {
        Material material = AssetDatabase.LoadAssetAtPath<Material>(MaterialPath);
        Shader shader = Shader.Find("Standard");
        if (shader == null)
            shader = Shader.Find("Universal Render Pipeline/Lit");
        if (shader == null)
            throw new InvalidOperationException("No supported lit shader is available");
        if (material == null)
        {
            material = new Material(shader);
            material.name = "RM75_Normal";
            AssetDatabase.CreateAsset(material, MaterialPath);
        }
        else
        {
            material.shader = shader;
        }
        material.color = new Color(0.75f, 0.75f, 0.75f, 1f);
        EditorUtility.SetDirty(material);
        return material;
    }

    static JointDefinition ReadJoint(XElement element)
    {
        XElement origin = element.Element("origin");
        XElement axis = element.Element("axis");
        Vector3 rosPosition = ParseVector(RequiredAttribute(origin, "xyz"));
        Vector3 rosRpy = ParseVector(RequiredAttribute(origin, "rpy"));
        Vector3 rosAxis = ParseVector(RequiredAttribute(axis, "xyz"));
        Vector3 unityAxis = RM75StlImporter.ConvertVector(rosAxis);
        if (unityAxis.sqrMagnitude < 1e-8f)
            throw new InvalidDataException("Joint axis cannot be zero");
        return new JointDefinition
        {
            Name = RequiredAttribute(element, "name"),
            Parent = RequiredAttribute(element.Element("parent"), "link"),
            Child = RequiredAttribute(element.Element("child"), "link"),
            Position = RM75StlImporter.ConvertVector(rosPosition),
            Rotation = ConvertRotation(rosRpy),
            Axis = unityAxis.normalized
        };
    }

    static Quaternion ConvertRotation(Vector3 rpy)
    {
        double cr = Math.Cos(rpy.x), sr = Math.Sin(rpy.x);
        double cp = Math.Cos(rpy.y), sp = Math.Sin(rpy.y);
        double cy = Math.Cos(rpy.z), sy = Math.Sin(rpy.z);
        double[,] ros =
        {
            { cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr },
            { sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr },
            { -sp, cp * sr, cp * cr }
        };
        int[] swap = { 0, 2, 1 };
        var unity = new double[3, 3];
        for (int row = 0; row < 3; row++)
            for (int column = 0; column < 3; column++)
                unity[row, column] = ros[swap[row], swap[column]];
        Vector3 up = new Vector3((float)unity[0, 1], (float)unity[1, 1],
                                 (float)unity[2, 1]);
        Vector3 forward = new Vector3((float)unity[0, 2], (float)unity[1, 2],
                                      (float)unity[2, 2]);
        return Quaternion.LookRotation(forward, up);
    }

    static Vector3 ParseVector(string text)
    {
        string[] parts = text.Split((char[])null, StringSplitOptions.RemoveEmptyEntries);
        if (parts.Length != 3)
            throw new InvalidDataException("Expected a three-component URDF vector: " + text);
        return new Vector3(ParseFloat(parts[0]), ParseFloat(parts[1]), ParseFloat(parts[2]));
    }

    static float ParseFloat(string text)
    {
        return Single.Parse(text, NumberStyles.Float, CultureInfo.InvariantCulture);
    }

    static string RequiredAttribute(XElement element, string name)
    {
        if (element == null || element.Attribute(name) == null)
            throw new InvalidDataException("URDF element is missing attribute " + name);
        return element.Attribute(name).Value;
    }

    static string MeshPath(string link)
    {
        return MeshDirectory + "/" + link + ".STL";
    }

    static void EnsureAssetFolders()
    {
        Directory.CreateDirectory(Path.GetFullPath(Path.GetDirectoryName(MaterialPath)));
        Directory.CreateDirectory(Path.GetFullPath(Path.GetDirectoryName(PrefabPath)));
        AssetDatabase.Refresh();
    }

    static void SetObjectArray<T>(UnityEngine.Object target, string propertyName, T[] values)
        where T : UnityEngine.Object
    {
        var data = new SerializedObject(target);
        SerializedProperty property = data.FindProperty(propertyName);
        property.arraySize = values.Length;
        for (int index = 0; index < values.Length; index++)
            property.GetArrayElementAtIndex(index).objectReferenceValue = values[index];
        data.ApplyModifiedPropertiesWithoutUndo();
    }

    static void SetVectorArray(UnityEngine.Object target, string propertyName, Vector3[] values)
    {
        var data = new SerializedObject(target);
        SerializedProperty property = data.FindProperty(propertyName);
        property.arraySize = values.Length;
        for (int index = 0; index < values.Length; index++)
            property.GetArrayElementAtIndex(index).vector3Value = values[index];
        data.ApplyModifiedPropertiesWithoutUndo();
    }
}
#endif
