/**
 * @module planning/utils
 * @description Utility functions for the planning and navigation module
 */

import type { Vector2, Pose } from '../../robotics/drone/types';

/**
 * Normalizes an angle to the range [-PI, PI]
 */
export function normalizeAngle(angle: number): number {
    let normalized = angle % (2 * Math.PI);
    if (normalized > Math.PI) normalized -= 2 * Math.PI;
    if (normalized < -Math.PI) normalized += 2 * Math.PI;
    return normalized;
}

/**
 * Computes Euclidean distance between two points in 2D space
 */
export function distance(p1: Vector2, p2: Vector2): number {
    return Math.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2);
}

/**
 * Checks if the robot has reached a specific goal within a given threshold
 */
export function goalReached(currentPose: Pose, goal: Pose, threshold: number = 0.1): boolean {
    const dist = Math.sqrt((currentPose.x - goal.x) ** 2 + (currentPose.y - goal.y) ** 2);
    return dist < threshold;
}

// ==================== Min Heap Priority Queue ====================

/**
 * Entry in the priority queue with a string key and numeric priority.
 */
export interface MinHeapEntry {
    key: string;
    priority: number;
}

/**
 * Generic min-heap priority queue for A* search algorithms.
 * 
 * This class provides O(log n) push and pop operations for
 * priority-based graph search algorithms.
 * 
 * @example
 * ```typescript
 * const heap = new MinHeap();
 * heap.push('node1', 5.0);
 * heap.push('node2', 3.0);
 * const min = heap.pop(); // { key: 'node2', priority: 3.0 }
 * ```
 */
export class MinHeap {
    private heap: MinHeapEntry[] = [];

    /** Add a new entry to the heap */
    push(key: string, priority: number): void {
        this.heap.push({ key, priority });
        this.bubbleUp(this.heap.length - 1);
    }

    /** Remove and return the entry with lowest priority */
    pop(): MinHeapEntry | undefined {
        if (this.heap.length === 0) return undefined;
        const min = this.heap[0];
        const last = this.heap.pop()!;
        if (this.heap.length > 0) {
            this.heap[0] = last;
            this.bubbleDown(0);
        }
        return min;
    }

    /** Check if the heap is empty */
    isEmpty(): boolean {
        return this.heap.length === 0;
    }

    /** Get the current size of the heap */
    size(): number {
        return this.heap.length;
    }

    private bubbleUp(index: number): void {
        while (index > 0) {
            const parent = Math.floor((index - 1) / 2);
            if (this.heap[parent].priority <= this.heap[index].priority) break;
            [this.heap[parent], this.heap[index]] = [this.heap[index], this.heap[parent]];
            index = parent;
        }
    }

    private bubbleDown(index: number): void {
        while (true) {
            const left = 2 * index + 1;
            const right = 2 * index + 2;
            let smallest = index;
            if (left < this.heap.length && this.heap[left].priority < this.heap[smallest].priority) {
                smallest = left;
            }
            if (right < this.heap.length && this.heap[right].priority < this.heap[smallest].priority) {
                smallest = right;
            }
            if (smallest === index) break;
            [this.heap[smallest], this.heap[index]] = [this.heap[index], this.heap[smallest]];
            index = smallest;
        }
    }
}

/**
 * Generic priority queue entry for use in search algorithms.
 */
export interface PriorityQueueEntry<T = string> {
    item: T;
    priority: number;
}

/**
 * Generic min-heap priority queue that can store any item type.
 * 
 * @example
 * ```typescript
 * interface Node { x: number; y: number; }
 * const pq = new PriorityQueue<Node>();
 * pq.push({ x: 1, y: 2 }, 5.0);
 * const { item, priority } = pq.pop()!;
 * ```
 */
export class PriorityQueue<T> {
    private heap: PriorityQueueEntry<T>[] = [];

    push(item: T, priority: number): void {
        this.heap.push({ item, priority });
        this.bubbleUp(this.heap.length - 1);
    }

    pop(): PriorityQueueEntry<T> | undefined {
        if (this.heap.length === 0) return undefined;
        const min = this.heap[0];
        const last = this.heap.pop()!;
        if (this.heap.length > 0) {
            this.heap[0] = last;
            this.bubbleDown(0);
        }
        return min;
    }

    isEmpty(): boolean {
        return this.heap.length === 0;
    }

    size(): number {
        return this.heap.length;
    }

    private bubbleUp(index: number): void {
        while (index > 0) {
            const parent = Math.floor((index - 1) / 2);
            if (this.heap[parent].priority <= this.heap[index].priority) break;
            [this.heap[parent], this.heap[index]] = [this.heap[index], this.heap[parent]];
            index = parent;
        }
    }

    private bubbleDown(index: number): void {
        while (true) {
            const left = 2 * index + 1;
            const right = 2 * index + 2;
            let smallest = index;
            if (left < this.heap.length && this.heap[left].priority < this.heap[smallest].priority) {
                smallest = left;
            }
            if (right < this.heap.length && this.heap[right].priority < this.heap[smallest].priority) {
                smallest = right;
            }
            if (smallest === index) break;
            [this.heap[smallest], this.heap[index]] = [this.heap[index], this.heap[smallest]];
            index = smallest;
        }
    }
}
