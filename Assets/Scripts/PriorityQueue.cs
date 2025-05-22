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
            // left child
            int childIndex = parentIndex * 2 + 1;

            // if there is no left child, stop sorting
            if (childIndex > lastIndex) break;

            // right child
            int rightchild = childIndex + 1;

            // if the value of the right child is less than the left child, switch to the right branch of the heap
            if (rightchild <= lastIndex && data[rightchild].CompareTo(data[childIndex]) < 0) childIndex = rightchild;

            // if the parent and child are already sorted, then stop sorting
            if (data[parentIndex].CompareTo(data[childIndex]) <= 0) break;

            // if not, then swap the parent and child
            T tmp = data[parentIndex];
            data[parentIndex] = data[childIndex];
            data[childIndex] = tmp;

            // move down the heap onto the child's level and repeat until sorted
            parentIndex = childIndex;
        }
        return frontItem;
    }
}