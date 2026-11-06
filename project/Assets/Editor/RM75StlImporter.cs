#if UNITY_EDITOR
using System;
using System.Collections.Generic;
using System.IO;
using UnityEditor.AssetImporters;
using UnityEngine;
using UnityEngine.Rendering;

[ScriptedImporter(1, "stl")]
public sealed class RM75StlImporter : ScriptedImporter
{
    public override void OnImportAsset(AssetImportContext context)
    {
        byte[] data = File.ReadAllBytes(context.assetPath);
        if (data.Length < 84)
            throw new InvalidDataException("Binary STL is shorter than its header: " + context.assetPath);

        uint triangleCount = BitConverter.ToUInt32(data, 80);
        long expectedLength = 84L + triangleCount * 50L;
        if (triangleCount == 0 || expectedLength != data.Length)
            throw new InvalidDataException("Invalid binary STL triangle table: " + context.assetPath);
        if (triangleCount > Int32.MaxValue / 3)
            throw new InvalidDataException("STL has too many triangles: " + context.assetPath);

        int vertexCount = checked((int)triangleCount * 3);
        var vertices = new List<Vector3>(vertexCount);
        var normals = new List<Vector3>(vertexCount);
        var triangles = new int[vertexCount];
        int offset = 84;
        for (int triangle = 0; triangle < triangleCount; triangle++)
        {
            Vector3 sourceNormal = ReadVector(data, offset);
            RequireFinite(sourceNormal, context.assetPath);
            Vector3 normal = ConvertVector(sourceNormal).normalized;
            offset += 12;
            int first = vertices.Count;
            for (int corner = 0; corner < 3; corner++)
            {
                Vector3 vertex = ConvertVector(ReadVector(data, offset));
                offset += 12;
                RequireFinite(vertex, context.assetPath);
                vertices.Add(vertex);
                normals.Add(normal);
            }
            offset += 2;

            // Swapping ROS Y/Z changes handedness. Reverse winding to retain
            // the source mesh's outward-facing surface.
            triangles[first] = first;
            triangles[first + 1] = first + 2;
            triangles[first + 2] = first + 1;
        }

        var mesh = new Mesh();
        mesh.name = Path.GetFileNameWithoutExtension(context.assetPath);
        if (vertexCount > UInt16.MaxValue)
            mesh.indexFormat = IndexFormat.UInt32;
        mesh.SetVertices(vertices);
        mesh.SetNormals(normals);
        mesh.SetTriangles(triangles, 0, true);
        mesh.RecalculateBounds();
        if (mesh.bounds.size.sqrMagnitude < 1e-12f)
            throw new InvalidDataException("STL has degenerate bounds: " + context.assetPath);
        mesh.UploadMeshData(true);
        context.AddObjectToAsset("mesh", mesh);
        context.SetMainObject(mesh);
    }

    static Vector3 ReadVector(byte[] data, int offset)
    {
        return new Vector3(BitConverter.ToSingle(data, offset),
                           BitConverter.ToSingle(data, offset + 4),
                           BitConverter.ToSingle(data, offset + 8));
    }

    internal static Vector3 ConvertVector(Vector3 ros)
    {
        return new Vector3(ros.x, ros.z, ros.y);
    }

    static void RequireFinite(Vector3 value, string path)
    {
        if (Single.IsNaN(value.x) || Single.IsInfinity(value.x) ||
            Single.IsNaN(value.y) || Single.IsInfinity(value.y) ||
            Single.IsNaN(value.z) || Single.IsInfinity(value.z))
            throw new InvalidDataException("STL contains a non-finite vertex: " + path);
    }
}
#endif
