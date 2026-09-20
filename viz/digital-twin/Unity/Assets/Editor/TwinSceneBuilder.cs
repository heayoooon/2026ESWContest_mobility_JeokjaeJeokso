using System;
using System.IO;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

public static class TwinSceneBuilder
{
    private static bool CargoOnly => File.Exists("Assets/Scan/cargo.obj");
    [MenuItem("Digital Twin/Build Scan Scene")]
    public static void Build()
    {
        string modelPath = CargoOnly ? "Assets/Scan/cargo.obj" : "Assets/Scan/cart.obj";
        AssetDatabase.Refresh();
        var source = AssetDatabase.LoadAssetAtPath<GameObject>(modelPath);
        if (source == null)
            throw new InvalidOperationException("Import the reconstructed cart OBJ and textures into Assets/Scan first.");

        var scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
        var scan = (GameObject)PrefabUtility.InstantiatePrefab(source);
        scan.name = CargoOnly ? "Cargo Only - Uncalibrated Scale" : "Cart Scan - Uncalibrated Scale";
        var renderers = scan.GetComponentsInChildren<Renderer>();
        if (renderers.Length == 0) throw new InvalidOperationException("The scan contains no renderable mesh.");
        var bounds = renderers[0].bounds;
        foreach (var renderer in renderers) bounds.Encapsulate(renderer.bounds);
        if (bounds.size.sqrMagnitude < 0.000001f) throw new InvalidOperationException("The scan bounds are empty.");
        Directory.CreateDirectory("Assets/ScanMaterials");
        foreach (var renderer in renderers)
        {
            var materials = renderer.sharedMaterials;
            for (int i = 0; i < materials.Length; i++)
            {
                if (materials[i] == null) continue;
                var path = "Assets/ScanMaterials/" + materials[i].name + ".mat";
                var material = AssetDatabase.LoadAssetAtPath<Material>(path);
                if (material == null)
                {
                    material = new Material(Shader.Find("Unlit/Texture"));
                    material.mainTexture = materials[i].mainTexture;
                    AssetDatabase.CreateAsset(material, path);
                }
                materials[i] = material;
            }
            renderer.sharedMaterials = materials;
        }

        var cameraObject = new GameObject("Inspection Camera", typeof(Camera), typeof(ScanOrbit));
        var camera = cameraObject.GetComponent<Camera>();
        camera.backgroundColor = new Color(0.12f, 0.14f, 0.15f);
        camera.clearFlags = CameraClearFlags.SolidColor;
        camera.fieldOfView = 45;
        var radius = bounds.extents.magnitude;
        camera.nearClipPlane = Mathf.Max(radius / 1000f, 0.001f);
        camera.farClipPlane = radius * 100;
        var orbit = cameraObject.GetComponent<ScanOrbit>();
        orbit.target = bounds.center;
        orbit.distance = radius * 2.7f;
        orbit.yaw = -35;
        orbit.minDistance = radius * 0.1f;
        orbit.maxDistance = radius * 20;
        orbit.ApplyPose();

        var light = new GameObject("Key Light", typeof(Light)).GetComponent<Light>();
        light.type = LightType.Directional;
        light.intensity = 1.2f;
        light.transform.rotation = Quaternion.Euler(45, -35, 0);
        RenderSettings.ambientLight = new Color(0.65f, 0.65f, 0.65f);
        Directory.CreateDirectory("Assets/Scenes");
        EditorSceneManager.SaveScene(scene, "Assets/Scenes/CartTwin.unity");
        EditorBuildSettings.scenes = new[] { new EditorBuildSettingsScene("Assets/Scenes/CartTwin.unity", true) };
        AssetDatabase.SaveAssets();
    }

    public static void Complete()
    {
        Build();
        RenderPreview();
        BuildPlayer();
    }

    public static void RenderPreview()
    {
        EditorSceneManager.OpenScene("Assets/Scenes/CartTwin.unity");
        var camera = UnityEngine.Object.FindFirstObjectByType<Camera>();
        var target = new RenderTexture(1600, 1000, 24);
        camera.targetTexture = target;
        camera.Render();
        var previous = RenderTexture.active;
        RenderTexture.active = target;
        var image = new Texture2D(1600, 1000, TextureFormat.RGB24, false);
        image.ReadPixels(new Rect(0, 0, 1600, 1000), 0, 0);
        image.Apply();
        Directory.CreateDirectory("Previews");
        File.WriteAllBytes(CargoOnly ? "Previews/cargo.png" : "Previews/cart.png", image.EncodeToPNG());
        camera.targetTexture = null;
        RenderTexture.active = previous;
        UnityEngine.Object.DestroyImmediate(image);
        target.Release();
        UnityEngine.Object.DestroyImmediate(target);
    }

    public static void BuildPlayer()
    {
        PlayerSettings.productName = CargoOnly ? "Cargo Only - Uncalibrated Scan" : "Cart Twin - Uncalibrated Scan";
        PlayerSettings.defaultIsNativeResolution = false;
        PlayerSettings.defaultScreenWidth = 1280;
        PlayerSettings.defaultScreenHeight = 800;
        PlayerSettings.fullScreenMode = FullScreenMode.Windowed;
        var report = BuildPipeline.BuildPlayer(new BuildPlayerOptions {
            scenes = new[] { "Assets/Scenes/CartTwin.unity" },
            locationPathName = CargoOnly ? "Builds/CargoOnly.exe" : "Builds/CartTwin.exe",
            target = BuildTarget.StandaloneWindows64,
            options = BuildOptions.None
        });
        if (report.summary.result != UnityEditor.Build.Reporting.BuildResult.Succeeded)
            throw new InvalidOperationException("Player build failed: " + report.summary.result);
    }
}
