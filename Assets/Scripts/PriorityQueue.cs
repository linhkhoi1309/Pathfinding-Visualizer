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
        BubbleUp(data.Count - 1);
    }

    public T Dequeue()
    {
        if (data.Count == 0) throw new InvalidOperationException("Queue is empty");
        
        T frontItem = data[0];
        int lastIndex = data.Count - 1;
        data[0] = data[lastIndex];
        data.RemoveAt(lastIndex);
        
        if (data.Count > 0)
            BubbleDown(0);
            
        return frontItem;
    }

    private int IndexOf(T item)
    {
        for (int i = 0; i < data.Count; i++)
        {
            if (data[i].Equals(item))
                return i;
        }
        return -1;
    }

    // Update the priority of an existing item and maintain heap property
    public bool UpdatePriority(T item)
    {
        int index = IndexOf(item);
        if (index == -1) return false; // Item not found
        BubbleUp(index);
        BubbleDown(index);
        return true;
    }

    // Bubble up from given index
    private void BubbleUp(int childIndex)
    {
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

    // Bubble down from given index
    private void BubbleDown(int parentIndex)
    {
        int lastIndex = data.Count - 1;

        while (true)
        {
            int childIndex = parentIndex * 2 + 1;
            if (childIndex > lastIndex) break;

            int rightChild = childIndex + 1;
            if (rightChild <= lastIndex && data[rightChild].CompareTo(data[childIndex]) < 0)
                childIndex = rightChild;

            if (data[parentIndex].CompareTo(data[childIndex]) <= 0) break;

            T tmp = data[parentIndex];
            data[parentIndex] = data[childIndex];
            data[childIndex] = tmp;
            parentIndex = childIndex;
        }
    }
}