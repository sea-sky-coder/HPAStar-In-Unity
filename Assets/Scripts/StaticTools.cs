using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace SSGame.Tools
{
    /// <summary>
    /// 静态辅助函数
    /// </summary>
    public class StaticTools
    {

        /// <summary>
        /// 二分法定位插入下标
        /// </summary>
        /// <typeparam name="T"></typeparam>
        /// <param name="array">要查找的数组</param>
        /// <param name="e">要插入的元素</param>
        /// <param name="order">指示数组为顺序(从小到大)还是逆序(从大到小)排列,默认顺序(true)</param>
        /// <param name="start">起始定位下标</param>
        /// <param name="end">终止定位下标,负数表示数组长度</param>
        /// <returns>返回值表示应当插入的下标,-1表示查找失败</returns>
        public static int DichotomyIndex<T>(List<T> array, T e, bool order = true, int start = 0, int end = -1) where T : IComparer<T>
        {
            if (end < 0) end = array.Count;

            if (end == start) return start;
            else if (end < start) return -1;

            int index = (start + end) / 2;

            if (e.Compare(e, array[index]) > 0)
            {
                if (order) start = index + 1;
                else end = index;
            }
            else
            {
                if (order) end = index;
                else start = index + 1;
            }

            return DichotomyIndex(array, e, order, start, end);
        }

        /// <summary>
        /// 获取集合中所缺少的最小值
        /// </summary>
        /// <param name="set">要查找的集合</param>
        /// <param name="min">限定查找的最小值</param>
        /// <returns>返回所缺少的最小值</returns>
        public static int GetMinLack(SortedSet<int> set, int min = 0)
        {
            if (set.Count == 0 || set.Min > min) return min;
            if (set.Max < min) return min;

            if (1f > (set.Max - set.Min) / (float)set.Count) return set.Max + 1;

            int value = min;
            foreach(int i in set)
            {
                if (i < value) continue;
                if (value == i) value++;
                else return value;
            }

            return set.Max + 1;
        }

    }

}