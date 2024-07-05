using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using SSGame.Tools;

namespace SSGame.Navigation
{
    /// <summary>
    /// 寻路者,采用分层A*寻路算法
    /// </summary>
    public class PathFinder
    {
        //存储地图通行代价
        List<List<int>> map = new();

        //最大花费,大于等于该值的区域被视为无法通行
        public int maxCost = 9999_9999;

        //最大连续入口,小于该值的连续入口会将其中点作为过渡点,反之,取其两端作为过渡点
        public uint maxContinuum = 6;

        //最高级的地图簇
        MapCluster mapCluster;

        /// <summary>
        /// 构造函数
        /// </summary>
        /// <param name="map">花费地图</param>
        /// <param name="hierarchies">层级划分,由低到高,代表该级子簇的数组尺寸,划分不整齐会导致未对齐错误;长度小于1会导致数组越界错误</param>
        /// <param name="defaultCost">默认花费</param>
        /// <param name="maxCost">最大花费,大于该值的区域被认为无法通行</param>
        /// <param name="maxContinuum">最大连续入口数,大于该值的连续入口会在两端生成过渡点,反之在中心生成一个过渡点</param>
        public PathFinder(List<List<int>> map,Vector2Int[] hierarchies, int defaultCost, int maxCost = 9999_9999, uint maxContinuum = 6)
        {
            this.maxContinuum = maxContinuum;
            this.maxCost = maxCost;
            this.map = map;

            MapCluster[][] lowerClusters = SplitMap(map, hierarchies[0], defaultCost, maxCost, maxContinuum);
            for (int i = 1; i < hierarchies.Length; i++) lowerClusters = SplitMap(lowerClusters, hierarchies[i]);
            mapCluster = new(lowerClusters, true);
        }

        /// <summary>
        /// 更新地图
        /// </summary>
        /// <param name="startPoint">更新区域的起始点</param>
        /// <param name="costs">更新后的花费</param>
        public void UpdateMap(Vector2Int startPoint, int[][] costs)
        {
            Vector2Int size = new(costs.Length, costs[0].Length);
            for(int i = 0; i < size.x; i++)
            {
                for(int j = 0; j < size.y; j++)
                {
                    map[startPoint.x + i][startPoint.y + j] = costs[i][j];
                }
            }
            //扩大范围以更新邻居簇
            startPoint -= new Vector2Int(1, 1);
            size += new Vector2Int(1, 1);
            mapCluster.UpdateMap(startPoint, size, maxCost, maxContinuum);
        }

        /// <summary>
        /// 清除指定点的缓冲区
        /// </summary>
        /// <param name="point">要清除缓冲区的点</param>
        /// <returns>返回结果表示是否清除成功</returns>
        public bool ClearBuff(Vector2Int point)
        {
            return mapCluster.ClearBuff(point);
        }

        /// <summary>
        /// 清空缓冲区
        /// </summary>
        public void ClearAllBuff()
        {
            mapCluster.ClearAllBuff();
        }

        /// <summary>
        /// 使用分层A*算法进行寻路
        /// </summary>
        /// <param name="startPoint">起点索引</param>
        /// <param name="endPoint">终点索引</param>
        /// <param name="isAbstract">是否只搜索抽象路径,默认false</param>
        /// <returns>返回搜索到的路径,搜索失败返回的路径将为null</returns>
        public ResultPath FindPath(Vector2Int startPoint, Vector2Int endPoint, bool isAbstract = false)
        {
            if (isAbstract) return mapCluster.GetAbstractPath(startPoint, endPoint);
            else return mapCluster.GetCompletePath(startPoint, endPoint);
        }

        /// <summary>
        /// 使用A*算法进行寻路
        /// </summary>
        /// <param name="map">要寻路的地图花费矩阵</param>
        /// <param name="startPoint">起始点</param>
        /// <param name="endPoint">终点</param>
        /// <param name="maxCost">最大花费,大于等于该花费的区域被视为无法通行</param>
        /// <param name="defaultCost">默认花费,用于计算估算成本</param>
        /// <param name="x">可搜索范围的起点x值</param>
        /// <param name="y">可搜索范围的起点y值</param>
        /// <param name="width">可搜索范围的宽度,应当大于0</param>
        /// <param name="height">可搜索范围的长度,应当大于0</param>
        /// <returns>返回路径拐点的集合,没有搜索到路径则返回列表长度为0</returns>
        /// <returns></returns>
        public static ResultPath FindPath(List<List<int>> map, Vector2Int startPoint, Vector2Int endPoint, int maxCost, int defaultCost = 1, int x = 0, int y = 0, int width = -1, int height = -1)
        {
            if (width < 0) width = map.Count;
            if (height < 0) height = map[0].Count;

            List<Vector2Int> path = new();

            List<PathNode> closeNodes = new();
            List<PathNode> openNodes = new();

            openNodes.Add(new PathNode(startPoint, null, Mathf.Abs(endPoint.x - startPoint.x) + Mathf.Abs(endPoint.y - startPoint.y) * defaultCost, 0));

            //临时函数,用于检查指定节点的相邻节点,node为指定节点,index为其相邻节点索引
            System.Action<PathNode, Vector2Int> CheckNeighbor = new((node, index) =>
            {
                int indexG = map[index.x][index.y];
                if (indexG >= maxCost) return;

                int g = node.g + indexG;
                //检查开放列表
                int i = openNodes.FindIndex(_ => _.index == index);
                if (i >= 0)
                {
                    PathNode temp = openNodes[i];
                    if (g < temp.g)
                    {
                        temp.f += g - temp.g;
                        temp.g = g;
                        temp.parent = node;
                        openNodes.RemoveAt(i);
                        int j = StaticTools.DichotomyIndex(openNodes, temp);
                        openNodes.Insert(j, temp);
                    }
                }
                else
                {
                    //检查封闭列表
                    if (closeNodes.Find(_ => _.index == index) == null)
                    {
                        PathNode temp = new(index, node, g + Mathf.Abs(endPoint.x - index.x) + Mathf.Abs(endPoint.y - index.y), g);
                        int j = StaticTools.DichotomyIndex(openNodes, temp);
                        openNodes.Insert(j, temp);
                    }
                }
            });

            //存储终点节点
            PathNode endNode = null;
            while (openNodes.Count > 0)
            {
                PathNode node = openNodes[0];
                openNodes.RemoveAt(0);
                if (node.index == endPoint)
                {
                    endNode = node;
                    break;
                }
                closeNodes.Add(node);

                //上
                Vector2Int index = new(node.index.x, node.index.y - 1);
                if (index.y >= y) CheckNeighbor(node, index);
                //右
                index = new(node.index.x + 1, node.index.y);
                if (index.x < x + width) CheckNeighbor(node, index);
                //下
                index = new(node.index.x, node.index.y + 1);
                if (index.y < y + height) CheckNeighbor(node, index);
                //左
                index = new(node.index.x - 1, node.index.y);
                if (index.x >= x) CheckNeighbor(node, index);
            }

            int g = 0;
            if (endNode != null)
            {
                g = endNode.g;
                if (endPoint != startPoint) path.Insert(0, endPoint);
                while (endNode.parent != null)
                {
                    Vector2Int lastIndex = path[0];
                    //检查共线情况
                    if (!((lastIndex.x == endNode.index.x && endNode.index.x == endNode.parent.index.x) || (lastIndex.y == endNode.index.y && endNode.index.y == endNode.parent.index.y)))
                    {
                        path.Insert(0, endNode.index);
                    }
                    endNode = endNode.parent;
                }
            }

            return new ResultPath(path, g);
        }

        /// <summary>
        /// 分割地图
        /// </summary>
        /// <param name="map">要分割的地图</param>
        /// <param name="size">分割单元的尺寸,必须能被地图尺寸整除,否则将抛出未对齐错误</param>
        /// <param name="defaultCost">默认花费</param>
        /// <param name="maxCost">最大花费,大于该值的区域被认为无法通行</param>
        /// <param name="maxContinuum">最大连续入口数</param>
        /// <returns>返回划分后的簇</returns>
        static MapCluster[][] SplitMap(List<List<int>> map, Vector2Int size, int defaultCost, int maxCost, uint maxContinuum)
        {
            if (map.Count % size.x != 0 || map[0].Count % size.y != 0) throw new UnalignedException("尺寸划分未与地图尺寸对齐");

            int width = map.Count / size.x;
            int height = map[0].Count / size.y;

            MapCluster[][] clusters = new MapCluster[width][];
            for(int i = 0; i < width; i++)
            {
                clusters[i] = new MapCluster[height];
                for(int j = 0; j < height; j++)
                {
                    clusters[i][j] = new(map, new(i * size.x, j * size.y), size, defaultCost, maxCost, maxContinuum);
                }
            }
            return clusters;
        }

        /// <summary>
        /// 分割地图(不可用于生成最高级簇)
        /// </summary>
        /// <param name="subClusters">子簇</param>
        /// <param name="size">分割成的数组尺寸(未对齐将抛出错误)</param>
        /// <returns>返回分割后的簇</returns>
        static MapCluster[][] SplitMap(MapCluster[][] subClusters, Vector2Int size)
        {
            if (subClusters.Length % size.x != 0 || subClusters[0].Length % size.y != 0) throw new UnalignedException("尺寸划分未与地图尺寸对齐");

            int width = subClusters.Length / size.x;
            int height = subClusters.Length / size.y;

            MapCluster[][] clusters = new MapCluster[width][];
            for(int i = 0; i < width; i++)
            {
                clusters[i] = new MapCluster[height];

                for(int j = 0; j < height; j++)
                {
                    MapCluster[][] pClusters = subClusters[(i * size.x)..((i + 1) * size.x)];
                    for (int k = 0; k < pClusters.Length; k++)
                    {
                        pClusters[k] = pClusters[k][(j * size.y)..((j + 1) * size.y)];
                    }
                    clusters[i][j] = new MapCluster(pClusters);
                }
            }

            return clusters;
        }
    }

    /// <summary>
    /// 存储寻路结果
    /// </summary>
    public struct ResultPath
    {
        //存储路径的拐点,长度为0时表示无法到达目的地
        public List<Vector2Int> path;
        //路径的实际花费
        public int g;

        public ResultPath(List<Vector2Int> path, int g)
        {
            this.path = path;
            this.g = g;
        }

        /// <summary>
        /// 反转路径
        /// </summary>
        /// <param name="map">该路径所处的地图</param>
        /// <param name="end">反转后路径的终点</param>
        /// <returns>返回反转后的路径</returns>
        public ResultPath Inverse(List<List<int>> map, Vector2Int end)
        {
            if (path == null || path.Count == 0) return new(path, g);

            List<Vector2Int> inversePath = new(path);
            inversePath.Reverse();
            inversePath.Add(end);
            Vector2Int start = inversePath[0];
            inversePath.RemoveAt(0);
            return new(inversePath, g - map[start.x][start.y] + map[end.x][end.y]);
        }
    }

    /// <summary>
    /// 未对齐错误,当地图的划分尺寸没有被地图尺寸整除时抛出该错误
    /// </summary>
    public class UnalignedException : System.ApplicationException
    {
        public UnalignedException(string error) : base(error)
        {
        }
    }

    /// <summary>
    /// 用于A*寻路的路径节点
    /// </summary>
    class PathNode : IComparer<PathNode>
    {
        //该节点在地图中的索引
        public Vector2Int index;
        //父节点,置空表示该节点为起点
        public PathNode parent;
        //路径总花费
        public int f;
        //路径实际花费
        public int g;

        public PathNode(Vector2Int index, PathNode parent, int f, int g)
        {
            this.index = index;
            this.parent = parent;
            this.f = f;
            this.g = g;
        }

        public static bool operator >(PathNode self, PathNode other)
        {
            return self.f > other.f;
        }

        public static bool operator <(PathNode self, PathNode other)
        {
            return self.f < other.f;
        }

        public int Compare(PathNode self, PathNode other)
        {
            if (self.f == other.f) return 0;
            if (self.f > other.f) return 1;
            return -1;
        }
    }

    /// <summary>
    /// 地图簇
    /// </summary>
    class MapCluster
    {
        //在地图中的起始下标
        Vector2Int startIndex;

        //该簇的尺寸(宽和高)
        Vector2Int size;

        //该簇使用的地图数据
        List<List<int>> map;

        //子簇,置空表示该簇为最低级的簇
        MapCluster[][] subClusters;

        //该簇的过渡点及其链接信息,置空表示该簇为最高级的簇(通常包含整个地图)
        Dictionary<Vector2Int, List<TransitionLink>> links;

        //默认花费
        public int defaultCost = 1;

        //最大花费
        public int maxCost = 9999_9999;

        //缓存指定点与过渡点的链接
        Dictionary<Vector2Int, List<TransitionLink>> linksBuff = new();

        /// <summary>
        /// 构造函数(构造最低级的簇)
        /// </summary>
        /// <param name="map">地图</param>
        /// <param name="startIndex">起始点在地图中的索引</param>
        /// <param name="size">簇的尺寸</param>
        /// <param name="defaultCost">默认花费</param>
        /// <param name="maxCost">最大花费,大于等于该值的被认为无法通行</param>
        /// <param name="maxContinuum">最大连续入口数量,大于该值的连续入口会将其两端作为过渡点</param>
        public MapCluster(List<List<int>> map, Vector2Int startIndex, Vector2Int size, int defaultCost, int maxCost, uint maxContinuum)
        {
            this.map = map;
            this.startIndex = startIndex;
            this.size = size;
            subClusters = null;
            this.defaultCost = defaultCost;
            this.maxCost = maxCost;

            links = GetTransitions(map, startIndex, size, defaultCost, maxCost, maxContinuum);
        }

        /// <summary>
        /// 构造函数,用于构造非最低级簇
        /// </summary>
        /// <param name="subClusters">子簇</param>
        /// <param name="isGreatest">指示该簇是否是最高级簇,最高级将不会创建过渡点</param>
        public MapCluster(MapCluster[][] subClusters, bool isGreatest = false)
        {
            MapCluster sample = subClusters[0][0];
            map = sample.map;
            startIndex = sample.startIndex;
            size = new(sample.size.x * subClusters.Length, sample.size.y * subClusters[0].Length);
            this.subClusters = subClusters;
            defaultCost = sample.defaultCost;
            maxCost = sample.maxCost;

            if (!isGreatest)
            {
                UpdateTransitionLinks();
            }
        }

        /// <summary>
        /// 更新地图
        /// </summary>
        /// <param name="startIndex">更新区域的起始索引</param>
        /// <param name="size">更新区域的尺寸</param>
        /// <param name="maxCost">最大花费,大于该值的区域被视为无法通行</param>
        /// <param name="maxContinumm">最大连续入口数</param>
        public void UpdateMap(Vector2Int startIndex, Vector2Int size, int maxCost, uint maxContinumm)
        {
            ClearAllBuff();
            if (subClusters == null)
            {
                links = GetTransitions(map, this.startIndex, this.size, defaultCost, maxCost, maxContinumm);
            }
            else
            {
                Vector2Int sIndex = LocatePoint(startIndex);
                Vector2Int eIndex = LocatePoint(new(startIndex.x + size.x - 1, startIndex.y + size.y - 1));

                for (int x = sIndex.x; x <= eIndex.x; x++)
                {
                    if (x < 0) continue;
                    if (x >= subClusters.Length) break;
                    for (int y = sIndex.y; y <= eIndex.y; y++)
                    {
                        if (y < 0) continue;
                        if (y >= subClusters[x].Length) break;
                        subClusters[x][y].UpdateMap(startIndex, size, maxCost, maxContinumm);
                    }
                }

                if (links != null) UpdateTransitionLinks();
            }
        }

        /// <summary>
        /// 获取指定起点到指定终点的抽象路径
        /// </summary>
        /// <param name="startPoint">起点在地图中的索引</param>
        /// <param name="endPoint">终点在地图中的索引</param>
        /// <returns>如果返回值中的路径为null则表示查找失败或无法从起点抵达终点</returns>
        public ResultPath GetAbstractPath(Vector2Int startPoint, Vector2Int endPoint)
        {
            if (!IsInCluster(startPoint) || !IsInCluster(endPoint)) return new(null, 0);
            if (endPoint == startPoint) return new(new(), 0);

            if (links != null)
            {
                if (links.TryGetValue(startPoint, out List<TransitionLink> sLinks))
                {
                    TransitionLink link = sLinks.Find(_ => _.index == endPoint);
                    if (link != null) return new ResultPath(link.path, link.g);
                }
                if (linksBuff.TryGetValue(startPoint, out sLinks))
                {
                    TransitionLink link = sLinks.Find(_ => _.index == endPoint);
                    if (link != null) return new(link.path, link.g);
                }
            }

            //构建终点与子簇的链接
            if (subClusters != null)
            {
                Vector2Int childIndex = LocatePoint(endPoint);
                List<TransitionLink> endLinks = subClusters[childIndex.x][childIndex.y].GetTransitionLinks(endPoint);
                if (endLinks.Count == 0) return new(null, 0);

                //判断起点与终点是否在同一子簇中
                Vector2Int subIndex = LocatePoint(startPoint);
                if (subIndex == LocatePoint(endPoint))
                {
                    MapCluster subCluster = subClusters[subIndex.x][subIndex.y];
                    ResultPath path = subCluster.subClusters == null ? 
                        PathFinder.FindPath(map, startPoint, endPoint, maxCost, defaultCost, subCluster.startIndex.x, subCluster.startIndex.y, subCluster.size.x, subCluster.size.y) 
                        : subCluster.GetAbstractPath(startPoint, endPoint);
                    if (path.path != null && path.path.Count > 0) return path;
                }
            }

            return subClusters == null ? PathFinder.FindPath(map, startPoint, endPoint, maxCost, defaultCost, startIndex.x, startIndex.y, size.x, size.y) : BridgePoints(startPoint, endPoint);
        }

        /// <summary>
        /// 获取完整的路径
        /// </summary>
        /// <param name="startPoint">起始点在地图中的索引</param>
        /// <param name="endPoint">终点在地图中的索引</param>
        /// <returns>返回查找到的路径,如果路径不存在则路径值为null</returns>
        public ResultPath GetCompletePath(Vector2Int startPoint, Vector2Int endPoint)
        {
            ResultPath abstractPath = GetAbstractPath(startPoint, endPoint);
            if (abstractPath.path == null || subClusters == null) return abstractPath;

            ResultPath resultPath = new(new(), 0);

            Vector2Int lastPoint = startPoint;
            foreach(Vector2Int point in abstractPath.path)
            {
                Vector2Int sIndex = LocatePoint(lastPoint);
                Vector2Int eIndex = LocatePoint(point);
                ResultPath path;
                if(sIndex == eIndex) path = subClusters[sIndex.x][sIndex.y].GetCompletePath(lastPoint, point);
                else path = BridgePoints(lastPoint, point);

                if (path.path == null) return path;

                resultPath.path.AddRange(path.path);
                resultPath.g += path.g;

                lastPoint = point;
            }

            return resultPath;
        }

        /// <summary>
        /// 尝试移除指定点的链接缓冲区
        /// </summary>
        /// <param name="point">要移除的点在地图中的索引</param>
        /// <returns>返回值表示是否移除成功</returns>
        public bool ClearBuff(Vector2Int point)
        {
            //过渡点无法使用此方法清除
            if (links.ContainsKey(point)) return false;

            if (linksBuff.Remove(point))
            {
                //记录空的链接
                List<Vector2Int> emptyLinks = new();
                foreach (var pair in linksBuff)
                {
                    int index = pair.Value.FindIndex(_ => _.index == point);
                    if (index >= 0)
                    {
                        pair.Value.RemoveAt(index);
                        if (pair.Value.Count == 0) emptyLinks.Add(pair.Key);
                    }
                }

                //移除空的链接
                foreach (Vector2Int p in emptyLinks)
                {
                    linksBuff.Remove(p);
                }

                //清除子簇的缓冲区
                if (subClusters != null)
                {
                    Vector2Int subIndex = LocatePoint(point);
                    subClusters[subIndex.x][subIndex.y].ClearBuff(point);
                }

                return true;
            }

            return false;
        }

        /// <summary>
        /// 清空链接缓冲区
        /// </summary>
        public void ClearAllBuff()
        {
            linksBuff.Clear();

            //清除子簇的缓冲区
            if (subClusters != null)
            {
                foreach(var clusters in subClusters)
                {
                    foreach(MapCluster cluster in clusters)
                    {
                        cluster.ClearAllBuff();
                    }
                }
            }
        }

        /// <summary>
        /// 判断一个点是否在该簇内
        /// </summary>
        /// <param name="point">要查询的点在地图中的索引</param>
        /// <returns>指定点是否在该簇内</returns>
        public bool IsInCluster(Vector2Int point)
        {
            return point.x >= startIndex.x && point.y >= startIndex.y && point.x < startIndex.x + size.x && point.y < startIndex.y + size.y;
        }

        /// <summary>
        /// 获取指定点到当前簇的所有过渡点的最短路径,查找到的路径会被缓存
        /// </summary>
        /// <param name="point">要查找的起点</param>
        /// <returns>返回以子簇过渡构点成路径的过渡链接列表,如果是最低级簇,则路径构成为拐点</returns>
        List<TransitionLink> GetTransitionLinks(Vector2Int point)
        {
            List<TransitionLink> result = new();

            //表示该点是否是过渡点
            bool isTransition = false;
            if (links != null)
            {
                if (links.TryGetValue(point, out List<TransitionLink> tLinks))
                {
                    result.AddRange(tLinks);
                    isTransition = true;
                }
            }
            if (linksBuff.TryGetValue(point, out List<TransitionLink> transLinks))
            {
                result.AddRange(transLinks);
                return result;
            }

            if (isTransition) return result;

            List<TransitionLink> buffLinks = new();
            if(subClusters == null)
            {
                foreach (var pair in links)
                {
                    ResultPath path = PathFinder.FindPath(map, point, pair.Key,maxCost, defaultCost, startIndex.x, startIndex.y, size.x, size.y);
                    if(path.path.Count > 0) buffLinks.Add(new TransitionLink(pair.Key, path.path, path.g));

                    ResultPath reverse = path.Inverse(map, point);
                    if (linksBuff.TryGetValue(pair.Key, out List<TransitionLink> value)) value.Add(new TransitionLink(point, reverse.path, reverse.g));
                    else linksBuff.Add(pair.Key, new() { new(point, reverse.path, reverse.g) });
                }
            }
            else if(links != null)
            {
                foreach (var pair in links)
                {
                    ResultPath path = BridgePoints(point, pair.Key);
                    if (path.path.Count == 0) continue;
                    buffLinks.Add(new(pair.Key, path.path, path.g));

                    ResultPath reverse = path.Inverse(map, point);
                    if (linksBuff.TryGetValue(pair.Key, out List<TransitionLink> value)) value.Add(new TransitionLink(point, reverse.path, reverse.g));
                    else linksBuff.Add(pair.Key, new() { new(point, reverse.path, reverse.g) });
                }
            }

            linksBuff.Add(point, buffLinks);
            result.AddRange(buffLinks);
            return result;
        }

        /// <summary>
        /// 更新过渡点及其链接(不可对最高或最低级簇使用)
        /// </summary>
        void UpdateTransitionLinks()
        {
            if (subClusters == null) return;

            //存储过渡点及其子簇链接
            Dictionary<Vector2Int, List<TransitionLink>> transitions = new();
            Rect rect = new(subClusters[0][0].startIndex, new(subClusters[0][0].size.x * subClusters.Length, subClusters[0][0].size.y * subClusters[0].Length));
            foreach(MapCluster[] clusters in subClusters)
            {
                foreach(MapCluster cluster in clusters)
                {
                    foreach(var pair in cluster.links)
                    {
                        Vector2Int p = pair.Key;
                        //检测点是否在边缘
                        if(p.x == rect.x || p.x == rect.xMax -1 || p.y == rect.y|| p.y == rect.yMax - 1)
                        {
                            transitions.Add(pair.Key, pair.Value);
                        }
                    }
                }
            }

            links = new();
            foreach(var pair in transitions)
            {
                List<TransitionLink> tLinks = new();
                foreach(TransitionLink link in pair.Value)
                {
                    if (rect.Contains(link.index)) break;
                    else tLinks.Add(link);
                }
                if(tLinks.Count > 0) links.Add(pair.Key, tLinks);
            }

            //与其他过渡点建立链接
            List<Vector2Int> visited = new();
            foreach(var pair in links)
            {
                visited.Add(pair.Key);
                foreach(var other in links)
                {
                    if (visited.Contains(other.Key)) continue;
                    ResultPath path = BridgePoints(pair.Key, other.Key);
                    if (path.path.Count == 0) continue;
                    if (path.path.Count > 0)
                    {
                        pair.Value.Add(new(other.Key, path.path, path.g));
                        ResultPath otherPath = path.Inverse(map, pair.Key);
                        other.Value.Add(new(pair.Key, otherPath.path, otherPath.g));
                    }
                }
            }
        }

        /// <summary>
        /// 采用基于节点的A*算法桥接两个点
        /// </summary>
        /// <param name="startPoint">起点索引</param>
        /// <param name="endPoint">终点索引</param>
        /// <returns>返回查找到的最短路径</returns>
        ResultPath BridgePoints(Vector2Int startPoint, Vector2Int endPoint)
        {
            List<PathNode> openNodes = new();
            List<PathNode> closeNodes = new();

            openNodes.Add(new(startPoint, null, (Mathf.Abs(endPoint.x - startPoint.x) + Mathf.Abs(endPoint.y - startPoint.y)) * defaultCost, 0));
            PathNode endNode = null;
            while (openNodes.Count > 0)
            {
                PathNode node = openNodes[0];
                openNodes.RemoveAt(0);
                if (node.index == endPoint)
                {
                    endNode = node;
                    break;
                }
                closeNodes.Add(node);

                Vector2Int clusterIndex = LocatePoint(node.index);
                List<TransitionLink> links = subClusters[clusterIndex.x][clusterIndex.y].GetTransitionLinks(node.index);

                foreach (TransitionLink link in links)
                {
                    Vector2Int index = link.index;
                    if (!IsInCluster(index)) continue;

                    int g = node.g + link.g;
                    //检查开放列表
                    int i = openNodes.FindIndex(_ => _.index == index);
                    if (i >= 0)
                    {
                        PathNode temp = openNodes[i];
                        if (g < temp.g)
                        {
                            temp.f += g - temp.g;
                            temp.g = g;
                            temp.parent = node;
                            openNodes.RemoveAt(i);
                            int j = StaticTools.DichotomyIndex(openNodes, temp);
                            openNodes.Insert(j, temp);
                        }
                    }
                    else
                    {
                        //检查封闭列表
                        if (closeNodes.Find(_ => _.index == index) == null)
                        {
                            PathNode temp = new(index, node, g + Mathf.Abs(endPoint.x - index.x) + Mathf.Abs(endPoint.y - index.y), g);
                            int j = StaticTools.DichotomyIndex(openNodes, temp);
                            openNodes.Insert(j, temp);
                        }
                    }
                }
            }

            int cost = 0;
            List<Vector2Int> path = new();
            if (endNode != null)
            {
                cost = endNode.g;
                if(endPoint != startPoint) path.Insert(0, endPoint);
                while (endNode.parent != null)
                {
                    endNode = endNode.parent;
                    path.Insert(0, endNode.index);
                }
            }

            return new(path, cost);
        }

        /// <summary>
        /// 获取指定区域的过渡点及其链接
        /// </summary>
        /// <param name="map">地图</param>
        /// <param name="startPoint">簇的起点</param>
        /// <param name="size">簇的尺寸</param>
        /// <param name="defaultCost">默认花费</param>
        /// <param name="maxCost">最大花费,任何大于等于该值的花费被认为无法通行</param>
        /// <param name="maxContinuum">最大连续体,大于等于该值的连续入口会记录两个过渡点,否则值将其终点作为过渡点</param>
        /// <returns>返回查找到的过渡点及其所有链接</returns>
        static Dictionary<Vector2Int, List<TransitionLink>> GetTransitions(List<List<int>> map,Vector2Int startPoint, Vector2Int size,int defaultCost, int maxCost, uint maxContinuum)
        {
            Dictionary<Vector2Int, List<TransitionLink>> links = new();

            //检查过渡点,参数依次为起始点,遍历方向,邻居点方向,迭代次数
            System.Action<Vector2Int, Vector2Int, Vector2Int, int> CheckTransition = new((start, dir, nei, count) =>
             {
                 RectInt rect = new(0, 0, map.Count, map[0].Count);
                 //记录上一个连续入口边界点
                 Vector2Int lastEntry = start;
                 //表示上一个是否是入口
                 bool last = false;
                 //当前遍历的点
                 Vector2Int cPoint = start;
                 for(int i = 0; i <= count; i++)
                 {
                     if(i < count && map[cPoint.x][cPoint.y] < maxCost && rect.Contains(cPoint + nei) && map[cPoint.x + nei.x][cPoint.y + nei.y] < maxCost)
                     {
                         //是入口
                         if (!last) lastEntry = cPoint;
                         last = true;
                     }
                     else
                     {
                         if (last)
                         {
                             List<Vector2Int> points = new();
                             if(Mathf.Abs(cPoint.x-lastEntry.x) < maxContinuum && Mathf.Abs(cPoint.y-lastEntry.y) < maxContinuum)
                             {
                                 points.Add((cPoint + lastEntry) / 2);
                                 
                             }
                             else
                             {
                                 points.Add(lastEntry);
                                 points.Add(cPoint - dir);
                             }
                             foreach(Vector2Int point in points)
                             {
                                 if(links.TryGetValue(point, out List<TransitionLink> value))
                                 {
                                     Vector2Int nPoint = point + nei;
                                     value.Add(new(nPoint, new() { nPoint }, map[nPoint.x][nPoint.y]));
                                 }
                                 else
                                 {
                                     Vector2Int nPoint = point + nei;
                                     links.Add(point, new() { new(nPoint, new() { nPoint }, map[nPoint.x][nPoint.y]) });
                                 }
                             }
                         }
                         last = false;
                     }

                     cPoint += dir;
                 }
             });

            //上边
            CheckTransition(startPoint, new(1, 0), new(0, -1), size.x);
            //右
            CheckTransition(new(startPoint.x + size.x - 1, startPoint.y), new(0, 1), new(1, 0), size.y);
            //下
            CheckTransition(new(startPoint.x, startPoint.y + size.y - 1), new(1, 0), new(0, 1), size.x);
            //左
            CheckTransition(startPoint, new(0, 1), new(-1, 0), size.y);

            //存储已经访问过的点
            List<Vector2Int> visited = new();
            //与其他过渡点建立链接
            foreach(var pair in links)
            {
                visited.Add(pair.Key);
                foreach(var other in links)
                {
                    if (visited.Contains(other.Key)) continue;
                    ResultPath path = PathFinder.FindPath(map, pair.Key, other.Key, maxCost, defaultCost, startPoint.x, startPoint.y, size.x, size.y);
                    if(path.path.Count > 0)
                    {
                        pair.Value.Add(new(other.Key, path.path, path.g));
                        ResultPath otherPath = path.Inverse(map, pair.Key);
                        other.Value.Add(new(pair.Key, otherPath.path, otherPath.g));
                    }
                }
            }

            return links;
        }

        /// <summary>
        /// 获取指定点在子簇中的位置
        /// </summary>
        /// <param name="point">要定位的点的索引</param>
        /// <returns>返回其所在子簇的索引,如果没有子簇,则返回其与起始索引的相对坐标</returns>
        Vector2Int LocatePoint(Vector2Int point)
        {
            if (subClusters == null) return point - startIndex;

            point -= startIndex;
            return new(point.x / subClusters[0][0].size.x, point.y / subClusters[0][0].size.y);
        }

        /// <summary>
        /// 过渡链接,存储与其他过渡点的链接信息
        /// </summary>
        class TransitionLink
        {
            //该过渡在地图中的索引
            public Vector2Int index;

            //与该过渡间路径的实际花费
            public int g;

            /// <summary>
            /// 与该过渡间路径的缓存;可以为空,这样将会在每次查找时搜索路径
            /// 最低级簇时,该值存储实际路径的拐点;其它级簇则存储过渡索引
            /// </summary>
            public List<Vector2Int> path;

            public TransitionLink(Vector2Int index, List<Vector2Int> path, int g)
            {
                this.index = index;
                this.path = path;
                this.g = g;
            }
        }
    }
}