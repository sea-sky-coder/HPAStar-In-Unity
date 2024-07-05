using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using SSGame.Tools;
using SSGame;
using SSGame.Navigation;
using UnityEngine.Events;
using UnityEditor;

public class Test : MonoBehaviour
{
    public Texture2D map;
    public int cellSize = 10;

    public Color gridColor = Color.black;
    public bool dashed = false;

    public Vector2Int[] hierachies;
    public RectInt[] updateObstacles;

    List<List<int>> cost;

    [ContextMenu("测试")]
    public void T()
    {
        var sw = new System.Diagnostics.Stopwatch();
        sw.Start();
        cost = PreprocessMap(out Vector2Int startPoint, out Vector2Int endPoint);
        sw.Stop();
        Debug.Log($"起点:{startPoint};终点:{endPoint}");
        Debug.Log($"Generate map cost time : {sw.ElapsedMilliseconds} ms");

        GeneratePathByA(startPoint, endPoint);
        GeneratePathByHPA(startPoint, endPoint);
    }

    /// <summary>
    /// 通过A*算法生成路径
    /// </summary>
    void GeneratePathByA(Vector2Int startPoint, Vector2Int endPoint)
    {
        var sw = new System.Diagnostics.Stopwatch();
        sw.Start();
        ResultPath path = PathFinder.FindPath(cost, startPoint, endPoint, maxCost: 9999_9999);
        sw.Stop();
        Debug.Log($"Path Finding cost time (A*) : {sw.Elapsed.TotalMilliseconds} ms");
        Debug.Log($"路径花费(A*):{path.g}");
        Texture2D pathMap = DrawPath(path.path, startPoint);
        System.IO.File.WriteAllBytes(AssetDatabase.GetAssetPath(map) + "A.png", pathMap.EncodeToPNG());
    }

    /// <summary>
    /// 通过HPA*算法生成路径
    /// </summary>
    void GeneratePathByHPA(Vector2Int startPoint, Vector2Int endPoint)
    {
        var sw = new System.Diagnostics.Stopwatch();
        sw.Start();
        PathFinder finder = new(cost, hierachies, 1);
        sw.Stop();
        Debug.Log($"Preprocess cost time : {sw.ElapsedMilliseconds} ms");
        sw = new System.Diagnostics.Stopwatch();
        UpdateMap(finder);
        Debug.Log($"Update map cost time : {sw.ElapsedMilliseconds} ms");
        sw = new System.Diagnostics.Stopwatch();
        sw.Start();
        ResultPath path = finder.FindPath(startPoint, endPoint);
        sw.Stop();
        Debug.Log($"Path Finding cost time (HPA*) : {sw.ElapsedMilliseconds} ms");
        Debug.Log($"路径花费(HPA*):{path.g}");
        Texture2D pathMap = DrawPath(path.path, startPoint);
        System.IO.File.WriteAllBytes(AssetDatabase.GetAssetPath(map) + "HPA.png", pathMap.EncodeToPNG());
    }

    /// <summary>
    /// 更新地图
    /// </summary>
    /// <param name="pathFinder"></param>
    void UpdateMap(PathFinder pathFinder)
    {
        foreach(RectInt rect in updateObstacles)
        {
            int[][] newCost = new int[rect.width][];
            for(int i = 0; i < rect.width; i++)
            {
                newCost[i] = new int[rect.height];
                for(int j = 0; j < rect.height; j++)
                {
                    newCost[i][j] = 9999_9999;
                }
            }

            pathFinder.UpdateMap(rect.position, newCost);
        }
    }

    /// <summary>
    /// 预处理地图
    /// </summary>
    /// <param name="startPoint"></param>
    /// <param name="endPoint"></param>
    /// <returns></returns>
    List<List<int>> PreprocessMap(out Vector2Int startPoint, out Vector2Int endPoint)
    {
        List<List<int>> cost = new();
        startPoint = new(-1, -1);
        endPoint = new(-1, -1);

        int width = Mathf.RoundToInt(map.width / cellSize);
        int height = Mathf.RoundToInt(map.height / cellSize);
        for(int i = 0; i < width; i++)
        {
            cost.Add(new());
            int x = i * cellSize + cellSize / 2;
            for(int j = 0; j < height; j++)
            {
                int y = j * cellSize + cellSize / 2;
                Color color = map.GetPixel(x, y);

                if(color.r > 0.9f || color.g > 0.9f || color.b > 0.9f)
                {
                    cost[^1].Add(1);
                    if(color.r > 0.9f && color.g < 0.1f && color.b < 0.1f)
                    {
                        endPoint = new(i, j);
                    }
                    else if(color.g > 0.9f && color.r < 0.1f && color.b < 0.1f)
                    {
                        startPoint = new(i, j);
                    }
                }
                else
                {
                    cost[^1].Add(9999_9999);
                }
            }
        }

        return cost;
    }

    /// <summary>
    /// 绘制地图
    /// </summary>
    /// <param name="path"></param>
    /// <param name="startPoint"></param>
    Texture2D DrawPath(List<Vector2Int> path, Vector2Int startPoint)
    {
        Texture2D tex = new Texture2D(map.width, map.height);
        tex.SetPixels(map.GetPixels());

        Vector2Int index = startPoint;
        foreach(Vector2Int point in path)
        {
            int sx = index.x * cellSize + cellSize / 2;
            int ex = point.x * cellSize + cellSize / 2;
            int sy = index.y * cellSize + cellSize / 2;
            int ey = point.y * cellSize + cellSize / 2;
            for (int x = Mathf.Min(sx, ex); x <= Mathf.Max(sx,ex); x++)
            {
                for(int y = Mathf.Min(sy, ey); y <= Mathf.Max(sy,ey); y++)
                {
                    tex.SetPixel(x, y, new(1, 1, 0));
                }
            }
            index = point;
        }

        return tex;
    }

    [ContextMenu("绘制细网格")]
    void DrawthinGrid()
    {
        Texture2D tex = new(map.width, map.height);
        tex.SetPixels(map.GetPixels());
        for(int i = cellSize; i < tex.width; i += cellSize)
        {
            for(int j = 0; j < tex.height; j++)
            {
                if (!dashed || j % 2 == 0)
                {
                    tex.SetPixel(i, j, gridColor);
                    tex.SetPixel(j, i, gridColor);
                }
            }
        }
        System.IO.File.WriteAllBytes(AssetDatabase.GetAssetPath(map) + "G.png", tex.EncodeToPNG());
    }
}
