using System;
using System.Collections.Generic;

public class PriorityQueue<T> where T : IComparable<T>
{
    private List<T> data;
    public int Count => data.Count;
    public bool Contains(T item) => data.Contains(item);
    public List<T> ToList() => data;
    public PriorityQueue() => data = new List<T>();

    public T Peek()
    {
        T front = data[0];
        return front;
    }

    public void Enqueue(T item)
    {
        data.Add(item);
        int childIndex = data.Count - 1;
        while (childIndex > 0)
        {
            int parentIndex = (childIndex - 1) / 2;
            if (data[childIndex].CompareTo(data[parentIndex]) >= 0) break;
            T tmp = data[childIndex];
            data[childIndex] = data[parentIndex];
            data[parentIndex] = tmp;
            childIndex = parentIndex;
        }
    }

    public T Dequeue()
    {
        int lastIndex = data.Count - 1;
        T frontItem = data[0];
        data[0] = data[lastIndex];
        data.RemoveAt(lastIndex);
        lastIndex--;
        int parentIndex = 0;

        while (true)
        {
            int childIndex = parentIndex * 2 + 1;
            if (childIndex > lastIndex) break;
            int rightchild = childIndex + 1;
            if (rightchild <= lastIndex && data[rightchild].CompareTo(data[childIndex]) < 0) childIndex = rightchild;
            if (data[parentIndex].CompareTo(data[childIndex]) <= 0) break;
            T tmp = data[parentIndex];
            data[parentIndex] = data[childIndex];
            data[childIndex] = tmp;
            parentIndex = childIndex;
        }
        return frontItem;
    }
}