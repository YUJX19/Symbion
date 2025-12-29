/**
 * @module planning/quickhull
 * @description QuickHull 3D Convex Hull Algorithm.
 * 
 * Efficient implementation of the QuickHull algorithm for computing
 * 3D convex hulls from point clouds. Used by geoutils for vertex
 * enumeration and safe flight corridor construction.
 * 
 * Features:
 * - O(n log n) average time complexity
 * - Handles degenerate cases (coplanar points, collinear points)
 * - Half-edge mesh output for easy traversal
 * - Configurable numerical tolerance
 * 
 * Based on Antti Kuukka's QuickHull implementation.
 * 
 * @example
 * ```typescript
 * import { QuickHull } from '@/lib/algorithms/planning/quickhull';
 * 
 * // Create hull computer
 * const qh = new QuickHull();
 * 
 * // Compute hull of cube vertices
 * const points = [
 *   [0, 0, 0], [1, 0, 0], [0, 1, 0], [1, 1, 0],
 *   [0, 0, 1], [1, 0, 1], [0, 1, 1], [1, 1, 1]
 * ];
 * const hull = qh.getConvexHull(points);
 * 
 * console.log(hull.vertices.length);   // 8
 * console.log(hull.faces.length);      // 12 triangular faces
 * ```
 */

// ==================== Vector3 ====================

/**
 * 3D Vector with basic operations.
 */
export class Vector3 {
    constructor(
        public x: number = 0,
        public y: number = 0,
        public z: number = 0
    ) { }

    /** Create from array [x, y, z] */
    static fromArray(arr: number[]): Vector3 {
        return new Vector3(arr[0] ?? 0, arr[1] ?? 0, arr[2] ?? 0);
    }

    /** Convert to array */
    toArray(): [number, number, number] {
        return [this.x, this.y, this.z];
    }

    /** Dot product */
    dot(other: Vector3): number {
        return this.x * other.x + this.y * other.y + this.z * other.z;
    }

    /** Cross product */
    cross(other: Vector3): Vector3 {
        return new Vector3(
            this.y * other.z - this.z * other.y,
            this.z * other.x - this.x * other.z,
            this.x * other.y - this.y * other.x
        );
    }

    /** Vector length */
    length(): number {
        return Math.sqrt(this.x * this.x + this.y * this.y + this.z * this.z);
    }

    /** Squared length */
    lengthSquared(): number {
        return this.x * this.x + this.y * this.y + this.z * this.z;
    }

    /** Normalize in place */
    normalize(): this {
        const len = this.length();
        if (len > 1e-12) {
            this.x /= len;
            this.y /= len;
            this.z /= len;
        }
        return this;
    }

    /** Get normalized copy */
    normalized(): Vector3 {
        const len = this.length();
        if (len > 1e-12) {
            return new Vector3(this.x / len, this.y / len, this.z / len);
        }
        return new Vector3(0, 0, 0);
    }

    /** Subtract vectors */
    sub(other: Vector3): Vector3 {
        return new Vector3(this.x - other.x, this.y - other.y, this.z - other.z);
    }

    /** Add vectors */
    add(other: Vector3): Vector3 {
        return new Vector3(this.x + other.x, this.y + other.y, this.z + other.z);
    }

    /** Scale vector */
    scale(s: number): Vector3 {
        return new Vector3(this.x * s, this.y * s, this.z * s);
    }

    /** Distance to another point */
    distanceTo(other: Vector3): number {
        return this.sub(other).length();
    }

    /** Squared distance */
    distanceSquaredTo(other: Vector3): number {
        return this.sub(other).lengthSquared();
    }

    /** Clone this vector */
    clone(): Vector3 {
        return new Vector3(this.x, this.y, this.z);
    }
}

// ==================== Plane ====================

/**
 * 3D Plane defined by normal and signed distance from origin.
 */
export class Plane {
    /** Normal vector */
    normal: Vector3;
    /** Signed distance from origin (if normal is unit length) */
    d: number;
    /** Squared length of normal */
    sqrNLength: number;

    constructor(normal: Vector3 = new Vector3(), point: Vector3 = new Vector3()) {
        this.normal = normal.clone();
        this.d = -normal.dot(point);
        this.sqrNLength = normal.lengthSquared();
    }

    /** Check if point is on positive side of plane */
    isPointOnPositiveSide(point: Vector3): boolean {
        return this.normal.dot(point) + this.d >= 0;
    }

    /** Get signed distance to point (not normalized) */
    signedDistanceTo(point: Vector3): number {
        return this.normal.dot(point) + this.d;
    }
}

// ==================== Half Edge Mesh ====================

/**
 * Half-edge in the mesh.
 */
interface HalfEdge {
    /** Index of end vertex */
    endVertex: number;
    /** Index of opposite half-edge */
    opp: number;
    /** Index of face this edge belongs to */
    face: number;
    /** Index of next half-edge in face */
    next: number;
    /** Is this edge disabled? */
    disabled: boolean;
}

/**
 * Face in the mesh.
 */
interface Face {
    /** Index of one half-edge of this face */
    he: number;
    /** Face plane */
    plane: Plane;
    /** Distance to most distant point */
    mostDistantPointDist: number;
    /** Index of most distant point */
    mostDistantPoint: number;
    /** Iteration when visibility was last checked */
    visibilityCheckedOnIteration: number;
    /** Is face visible from current point */
    isVisibleFaceOnCurrentIteration: boolean;
    /** Points on positive side of this face */
    pointsOnPositiveSide: number[];
    /** Is in face stack */
    inFaceStack: boolean;
    /** Which horizon edges on current iteration (bitmask) */
    horizonEdgesOnCurrentIteration: number;
    /** Is this face disabled? */
    disabled: boolean;
}

/**
 * Result of convex hull computation.
 */
export interface ConvexHullResult {
    /** Vertices of the hull */
    vertices: Vector3[];
    /** Face indices (each 3 indices form a triangle) */
    indices: number[];
    /** Face normals */
    normals: Vector3[];
}

// ==================== QuickHull Implementation ====================

/** Default epsilon for numerical comparisons */
export const defaultEpsilon = 1e-10;

/**
 * QuickHull 3D convex hull algorithm.
 */
export class QuickHull {
    private epsilon: number;
    private vertices: Vector3[] = [];
    private faces: Face[] = [];
    private halfEdges: HalfEdge[] = [];
    private disabledFaces: number[] = [];
    private disabledHalfEdges: number[] = [];
    private iteration: number = 0;

    /**
     * Create QuickHull instance.
     * @param epsilon Numerical tolerance
     */
    constructor(epsilon: number = defaultEpsilon) {
        this.epsilon = epsilon;
    }

    /**
     * Compute convex hull of point cloud.
     * 
     * @param points Input points as array of [x, y, z] arrays
     * @param ccw If true, output faces with counter-clockwise winding
     * @returns Convex hull result
     */
    getConvexHull(
        points: number[][] | Vector3[],
        ccw: boolean = true
    ): ConvexHullResult {
        // Convert to Vector3 if needed
        this.vertices = points.map(p =>
            p instanceof Vector3 ? p.clone() : Vector3.fromArray(p)
        );

        if (this.vertices.length < 4) {
            return { vertices: [...this.vertices], indices: [], normals: [] };
        }

        this.reset();

        // Find extreme points for initial tetrahedron
        const extremeResult = this.findExtremePoints();
        if (!extremeResult) {
            return { vertices: [...this.vertices], indices: [], normals: [] };
        }

        // Create initial tetrahedron
        if (!this.createInitialTetrahedron(extremeResult)) {
            return { vertices: [...this.vertices], indices: [], normals: [] };
        }

        // Main algorithm loop
        this.createConvexHull();

        // Build result
        return this.buildResult(ccw);
    }

    /**
     * Get convex hull from flat point array.
     * 
     * @param flatPoints Points as [x0, y0, z0, x1, y1, z1, ...]
     * @param ccw Counter-clockwise winding
     * @returns Convex hull
     */
    getConvexHullFromFlat(flatPoints: number[], ccw: boolean = true): ConvexHullResult {
        const points: number[][] = [];
        for (let i = 0; i < flatPoints.length; i += 3) {
            points.push([flatPoints[i], flatPoints[i + 1], flatPoints[i + 2]]);
        }
        return this.getConvexHull(points, ccw);
    }

    private reset(): void {
        this.faces = [];
        this.halfEdges = [];
        this.disabledFaces = [];
        this.disabledHalfEdges = [];
        this.iteration = 0;
    }

    private findExtremePoints(): { a: number; b: number; c: number; d: number } | null {
        const n = this.vertices.length;

        // Find extreme points in each axis
        let minX = 0, maxX = 0, minY = 0, maxY = 0, minZ = 0, maxZ = 0;

        for (let i = 1; i < n; i++) {
            const v = this.vertices[i];
            if (v.x < this.vertices[minX].x) minX = i;
            if (v.x > this.vertices[maxX].x) maxX = i;
            if (v.y < this.vertices[minY].y) minY = i;
            if (v.y > this.vertices[maxY].y) maxY = i;
            if (v.z < this.vertices[minZ].z) minZ = i;
            if (v.z > this.vertices[maxZ].z) maxZ = i;
        }

        // Find pair with maximum distance
        let maxDist = 0;
        let a = 0, b = 1;

        const extremes = [minX, maxX, minY, maxY, minZ, maxZ];
        for (let i = 0; i < extremes.length; i++) {
            for (let j = i + 1; j < extremes.length; j++) {
                const dist = this.vertices[extremes[i]].distanceSquaredTo(this.vertices[extremes[j]]);
                if (dist > maxDist) {
                    maxDist = dist;
                    a = extremes[i];
                    b = extremes[j];
                }
            }
        }

        if (maxDist < this.epsilon * this.epsilon) {
            return null; // Degenerate case
        }

        // Find point most distant from line a-b
        const ab = this.vertices[b].sub(this.vertices[a]);
        maxDist = 0;
        let c = 0;

        for (let i = 0; i < n; i++) {
            if (i === a || i === b) continue;
            const ap = this.vertices[i].sub(this.vertices[a]);
            const cross = ab.cross(ap);
            const dist = cross.lengthSquared();
            if (dist > maxDist) {
                maxDist = dist;
                c = i;
            }
        }

        if (maxDist < this.epsilon * this.epsilon) {
            return null; // Collinear
        }

        // Find point most distant from plane a-b-c
        const normal = this.getTriangleNormal(a, b, c);
        maxDist = 0;
        let d = 0;

        for (let i = 0; i < n; i++) {
            if (i === a || i === b || i === c) continue;
            const ap = this.vertices[i].sub(this.vertices[a]);
            const dist = Math.abs(normal.dot(ap));
            if (dist > maxDist) {
                maxDist = dist;
                d = i;
            }
        }

        if (maxDist < this.epsilon) {
            return null; // Coplanar
        }

        return { a, b, c, d };
    }

    private getTriangleNormal(a: number, b: number, c: number): Vector3 {
        const va = this.vertices[a];
        const vb = this.vertices[b];
        const vc = this.vertices[c];
        return vb.sub(va).cross(vc.sub(va));
    }

    private createInitialTetrahedron(extreme: { a: number; b: number; c: number; d: number }): boolean {
        const { a, d } = extreme;
        let { b, c } = extreme;

        // Ensure d is on positive side of plane abc
        const normal = this.getTriangleNormal(a, b, c);
        const ad = this.vertices[d].sub(this.vertices[a]);

        if (normal.dot(ad) > 0) {
            // Swap b and c to flip normal
            [b, c] = [c, b];
        }

        // Create faces
        this.faces = [];
        this.halfEdges = [];

        // Face 0: ABC
        // Face 1: ACD
        // Face 2: ADB
        // Face 3: BDC

        // Create half-edges (12 total for tetrahedron)
        for (let i = 0; i < 12; i++) {
            this.halfEdges.push({
                endVertex: 0,
                opp: 0,
                face: 0,
                next: 0,
                disabled: false
            });
        }

        // Setup half-edges for each face
        // Face 0: ABC (edges 0, 1, 2)
        this.halfEdges[0] = { endVertex: b, opp: 6, face: 0, next: 1, disabled: false };
        this.halfEdges[1] = { endVertex: c, opp: 9, face: 0, next: 2, disabled: false };
        this.halfEdges[2] = { endVertex: a, opp: 3, face: 0, next: 0, disabled: false };

        // Face 1: ACD (edges 3, 4, 5)
        this.halfEdges[3] = { endVertex: c, opp: 2, face: 1, next: 4, disabled: false };
        this.halfEdges[4] = { endVertex: d, opp: 11, face: 1, next: 5, disabled: false };
        this.halfEdges[5] = { endVertex: a, opp: 7, face: 1, next: 3, disabled: false };

        // Face 2: ADB (edges 6, 7, 8)
        this.halfEdges[6] = { endVertex: a, opp: 0, face: 2, next: 7, disabled: false };
        this.halfEdges[7] = { endVertex: d, opp: 5, face: 2, next: 8, disabled: false };
        this.halfEdges[8] = { endVertex: b, opp: 10, face: 2, next: 6, disabled: false };

        // Face 3: BDC (edges 9, 10, 11)
        this.halfEdges[9] = { endVertex: b, opp: 1, face: 3, next: 10, disabled: false };
        this.halfEdges[10] = { endVertex: d, opp: 8, face: 3, next: 11, disabled: false };
        this.halfEdges[11] = { endVertex: c, opp: 4, face: 3, next: 9, disabled: false };

        // Create faces with planes
        for (let i = 0; i < 4; i++) {
            const heIndex = i * 3;
            const v0 = this.halfEdges[this.halfEdges[heIndex].opp].endVertex;
            const v1 = this.halfEdges[heIndex].endVertex;
            const v2 = this.halfEdges[this.halfEdges[heIndex].next].endVertex;

            const normal = this.getTriangleNormal(v0, v1, v2);
            const plane = new Plane(normal, this.vertices[v0]);

            this.faces.push({
                he: heIndex,
                plane,
                mostDistantPointDist: 0,
                mostDistantPoint: 0,
                visibilityCheckedOnIteration: 0,
                isVisibleFaceOnCurrentIteration: false,
                pointsOnPositiveSide: [],
                inFaceStack: false,
                horizonEdgesOnCurrentIteration: 0,
                disabled: false
            });
        }

        // Assign points to faces
        const usedPoints = new Set([a, b, c, d]);

        for (let i = 0; i < this.vertices.length; i++) {
            if (usedPoints.has(i)) continue;

            for (let f = 0; f < 4; f++) {
                const dist = this.faces[f].plane.signedDistanceTo(this.vertices[i]);
                if (dist > this.epsilon) {
                    this.faces[f].pointsOnPositiveSide.push(i);
                    if (dist > this.faces[f].mostDistantPointDist) {
                        this.faces[f].mostDistantPointDist = dist;
                        this.faces[f].mostDistantPoint = i;
                    }
                    break;
                }
            }
        }

        return true;
    }

    private createConvexHull(): void {
        // Process faces that have points on positive side
        const faceStack: number[] = [];

        for (let i = 0; i < this.faces.length; i++) {
            if (this.faces[i].pointsOnPositiveSide.length > 0) {
                faceStack.push(i);
                this.faces[i].inFaceStack = true;
            }
        }

        while (faceStack.length > 0) {
            this.iteration++;
            const topFace = faceStack.pop()!;

            if (this.faces[topFace].disabled) continue;
            this.faces[topFace].inFaceStack = false;

            if (this.faces[topFace].pointsOnPositiveSide.length === 0) continue;

            // Get most distant point
            const pointIndex = this.faces[topFace].mostDistantPoint;
            const point = this.vertices[pointIndex];

            // Find visible faces from this point
            const visibleFaces: number[] = [];
            const horizonEdges: number[] = [];

            this.findVisibleFaces(topFace, point, visibleFaces);

            // Find horizon edges (edges where one face is visible, other is not)
            for (const faceIndex of visibleFaces) {
                const face = this.faces[faceIndex];
                const he0 = face.he;
                const he1 = this.halfEdges[he0].next;
                const he2 = this.halfEdges[he1].next;

                for (const heIndex of [he0, he1, he2]) {
                    const oppFace = this.halfEdges[this.halfEdges[heIndex].opp].face;
                    if (!visibleFaces.includes(oppFace) && !this.faces[oppFace].disabled) {
                        horizonEdges.push(heIndex);
                    }
                }
            }

            // Create new faces from horizon to point
            const newFaces = this.createNewFaces(horizonEdges, pointIndex);

            // Collect all points from visible faces
            const orphanPoints: number[] = [];
            for (const faceIndex of visibleFaces) {
                for (const p of this.faces[faceIndex].pointsOnPositiveSide) {
                    if (p !== pointIndex) {
                        orphanPoints.push(p);
                    }
                }
                this.disableFace(faceIndex);
            }

            // Assign orphan points to new faces
            for (const p of orphanPoints) {
                for (const faceIndex of newFaces) {
                    const face = this.faces[faceIndex];
                    const dist = face.plane.signedDistanceTo(this.vertices[p]);
                    if (dist > this.epsilon) {
                        face.pointsOnPositiveSide.push(p);
                        if (dist > face.mostDistantPointDist) {
                            face.mostDistantPointDist = dist;
                            face.mostDistantPoint = p;
                        }
                        break;
                    }
                }
            }

            // Add new faces with points to face stack
            for (const faceIndex of newFaces) {
                if (this.faces[faceIndex].pointsOnPositiveSide.length > 0 &&
                    !this.faces[faceIndex].inFaceStack) {
                    faceStack.push(faceIndex);
                    this.faces[faceIndex].inFaceStack = true;
                }
            }
        }
    }

    private findVisibleFaces(startFace: number, point: Vector3, visible: number[]): void {
        const stack = [startFace];
        this.faces[startFace].visibilityCheckedOnIteration = this.iteration;
        this.faces[startFace].isVisibleFaceOnCurrentIteration = true;
        visible.push(startFace);

        while (stack.length > 0) {
            const faceIndex = stack.pop()!;
            const face = this.faces[faceIndex];

            // Check adjacent faces
            const he0 = face.he;
            const he1 = this.halfEdges[he0].next;
            const he2 = this.halfEdges[he1].next;

            for (const heIndex of [he0, he1, he2]) {
                const oppFace = this.halfEdges[this.halfEdges[heIndex].opp].face;

                if (this.faces[oppFace].disabled) continue;
                if (this.faces[oppFace].visibilityCheckedOnIteration === this.iteration) continue;

                this.faces[oppFace].visibilityCheckedOnIteration = this.iteration;

                const dist = this.faces[oppFace].plane.signedDistanceTo(point);
                if (dist > this.epsilon) {
                    this.faces[oppFace].isVisibleFaceOnCurrentIteration = true;
                    visible.push(oppFace);
                    stack.push(oppFace);
                } else {
                    this.faces[oppFace].isVisibleFaceOnCurrentIteration = false;
                }
            }
        }
    }

    private createNewFaces(horizonEdges: number[], pointIndex: number): number[] {
        const newFaces: number[] = [];
        const newHalfEdges = new Map<number, number>(); // horizonEdge -> new edge pointing to point

        for (const heIndex of horizonEdges) {
            const he = this.halfEdges[heIndex];
            const startVertex = this.halfEdges[he.opp].endVertex;
            const endVertex = he.endVertex;

            // Create new face
            const faceIndex = this.addFace();
            newFaces.push(faceIndex);

            // Create 3 new half-edges
            const e0 = this.addHalfEdge(); // From startVertex to endVertex (along horizon)
            const e1 = this.addHalfEdge(); // From endVertex to point
            const e2 = this.addHalfEdge(); // From point to startVertex

            this.halfEdges[e0] = {
                endVertex: endVertex,
                opp: heIndex,
                face: faceIndex,
                next: e1,
                disabled: false
            };

            this.halfEdges[e1] = {
                endVertex: pointIndex,
                opp: -1, // Set later
                face: faceIndex,
                next: e2,
                disabled: false
            };

            this.halfEdges[e2] = {
                endVertex: startVertex,
                opp: -1, // Set later
                face: faceIndex,
                next: e0,
                disabled: false
            };

            // Update opposite reference
            this.halfEdges[heIndex].opp = e0;

            // Store for linking
            newHalfEdges.set(heIndex, e1);

            // Set face properties
            const normal = this.getTriangleNormal(startVertex, endVertex, pointIndex);
            this.faces[faceIndex].he = e0;
            this.faces[faceIndex].plane = new Plane(normal, this.vertices[startVertex]);
            this.faces[faceIndex].pointsOnPositiveSide = [];
            this.faces[faceIndex].mostDistantPointDist = 0;
        }

        // Link opposite edges between new faces
        for (let i = 0; i < horizonEdges.length; i++) {
            const heIndex = horizonEdges[i];
            const nextHeIndex = horizonEdges[(i + 1) % horizonEdges.length];

            // Find edges that should be opposite
            const e1 = newHalfEdges.get(heIndex)!;
            const e2Next = newHalfEdges.get(nextHeIndex)!;
            const e2 = this.halfEdges[e2Next].next; // Edge from point to start of next

            // Link if they share vertices correctly
            if (this.halfEdges[e1].endVertex === this.halfEdges[this.halfEdges[e2].opp]?.endVertex ||
                this.halfEdges[e1].endVertex === this.halfEdges[e2].endVertex) {
                this.halfEdges[e1].opp = e2;
                this.halfEdges[e2].opp = e1;
            }
        }

        return newFaces;
    }

    private addFace(): number {
        if (this.disabledFaces.length > 0) {
            const index = this.disabledFaces.pop()!;
            this.faces[index].disabled = false;
            return index;
        }

        this.faces.push({
            he: 0,
            plane: new Plane(),
            mostDistantPointDist: 0,
            mostDistantPoint: 0,
            visibilityCheckedOnIteration: 0,
            isVisibleFaceOnCurrentIteration: false,
            pointsOnPositiveSide: [],
            inFaceStack: false,
            horizonEdgesOnCurrentIteration: 0,
            disabled: false
        });

        return this.faces.length - 1;
    }

    private addHalfEdge(): number {
        if (this.disabledHalfEdges.length > 0) {
            const index = this.disabledHalfEdges.pop()!;
            this.halfEdges[index].disabled = false;
            return index;
        }

        this.halfEdges.push({
            endVertex: 0,
            opp: 0,
            face: 0,
            next: 0,
            disabled: false
        });

        return this.halfEdges.length - 1;
    }

    private disableFace(faceIndex: number): void {
        this.faces[faceIndex].disabled = true;
        this.disabledFaces.push(faceIndex);

        // Disable half-edges
        const he0 = this.faces[faceIndex].he;
        const he1 = this.halfEdges[he0].next;
        const he2 = this.halfEdges[he1].next;

        for (const heIndex of [he0, he1, he2]) {
            this.halfEdges[heIndex].disabled = true;
            this.disabledHalfEdges.push(heIndex);
        }
    }

    private buildResult(ccw: boolean): ConvexHullResult {
        const vertices: Vector3[] = [];
        const indices: number[] = [];
        const normals: Vector3[] = [];

        // Map from original vertex index to new index
        const vertexMap = new Map<number, number>();

        for (let f = 0; f < this.faces.length; f++) {
            if (this.faces[f].disabled) continue;

            const face = this.faces[f];
            const he0 = face.he;
            const he1 = this.halfEdges[he0].next;
            const he2 = this.halfEdges[he1].next;

            const v0 = this.halfEdges[this.halfEdges[he0].opp].endVertex;
            const v1 = this.halfEdges[he0].endVertex;
            const v2 = this.halfEdges[he1].endVertex;

            // Map vertices
            for (const v of [v0, v1, v2]) {
                if (!vertexMap.has(v)) {
                    vertexMap.set(v, vertices.length);
                    vertices.push(this.vertices[v].clone());
                }
            }

            // Add indices (with correct winding)
            if (ccw) {
                indices.push(vertexMap.get(v0)!, vertexMap.get(v1)!, vertexMap.get(v2)!);
            } else {
                indices.push(vertexMap.get(v0)!, vertexMap.get(v2)!, vertexMap.get(v1)!);
            }

            // Add normal
            normals.push(face.plane.normal.normalized());
        }

        return { vertices, indices, normals };
    }
}

// ==================== Utility Functions ====================

/**
 * Compute convex hull of 3D points.
 * Convenience function using default QuickHull instance.
 * 
 * @param points Input points
 * @param ccw Counter-clockwise winding
 * @returns Convex hull result
 */
export function computeConvexHull(
    points: number[][] | Vector3[],
    ccw: boolean = true
): ConvexHullResult {
    const qh = new QuickHull();
    return qh.getConvexHull(points, ccw);
}

/**
 * Check if points are coplanar.
 * 
 * @param points Input points
 * @param epsilon Tolerance
 * @returns true if all points lie on a plane
 */
export function arePointsCoplanar(points: Vector3[] | number[][], epsilon: number = 1e-8): boolean {
    if (points.length < 4) return true;

    const vecs = points.map(p => p instanceof Vector3 ? p : Vector3.fromArray(p));

    // Compute plane from first 3 non-collinear points
    const v0 = vecs[0];
    let v1: Vector3 | null = null;
    let v2: Vector3 | null = null;

    for (let i = 1; i < vecs.length && !v1; i++) {
        if (vecs[i].distanceTo(v0) > epsilon) {
            v1 = vecs[i];
        }
    }

    if (!v1) return true;

    const dir01 = v1.sub(v0).normalized();

    for (let i = 2; i < vecs.length && !v2; i++) {
        const dir0i = vecs[i].sub(v0).normalized();
        const cross = dir01.cross(dir0i);
        if (cross.length() > epsilon) {
            v2 = vecs[i];
        }
    }

    if (!v2) return true;

    // Check all points against plane
    const normal = v1.sub(v0).cross(v2.sub(v0)).normalized();
    const d = -normal.dot(v0);

    for (const v of vecs) {
        const dist = Math.abs(normal.dot(v) + d);
        if (dist > epsilon) return false;
    }

    return true;
}

/**
 * Compute centroid of convex hull.
 * 
 * @param hull Convex hull result
 * @returns Centroid position
 */
export function computeHullCentroid(hull: ConvexHullResult): Vector3 {
    if (hull.vertices.length === 0) return new Vector3();

    const centroid = new Vector3();
    for (const v of hull.vertices) {
        centroid.x += v.x;
        centroid.y += v.y;
        centroid.z += v.z;
    }

    const n = hull.vertices.length;
    centroid.x /= n;
    centroid.y /= n;
    centroid.z /= n;

    return centroid;
}

/**
 * Compute volume of convex hull.
 * Uses signed tetrahedron volumes from centroid.
 * 
 * @param hull Convex hull result
 * @returns Volume
 */
export function computeHullVolume(hull: ConvexHullResult): number {
    if (hull.indices.length < 3) return 0;

    const centroid = computeHullCentroid(hull);
    let volume = 0;

    for (let i = 0; i < hull.indices.length; i += 3) {
        const v0 = hull.vertices[hull.indices[i]];
        const v1 = hull.vertices[hull.indices[i + 1]];
        const v2 = hull.vertices[hull.indices[i + 2]];

        // Signed volume of tetrahedron
        const a = v0.sub(centroid);
        const b = v1.sub(centroid);
        const c = v2.sub(centroid);

        volume += Math.abs(a.dot(b.cross(c))) / 6;
    }

    return volume;
}
