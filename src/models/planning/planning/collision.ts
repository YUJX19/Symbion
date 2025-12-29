
import { Vector2, Robot } from './types';
import { Obstacle, WorldConfig } from '../../sensing/types';
import { distance } from './utils';

/**
 * Check collision between two circles
 */
export function circleCircleCollision(
    pos1: Vector2, radius1: number,
    pos2: Vector2, radius2: number
): boolean {
    const dx = pos2.x - pos1.x
    const dy = pos2.y - pos1.y
    const distSq = dx * dx + dy * dy
    const radiusSum = radius1 + radius2
    return distSq < radiusSum * radiusSum
}

/**
 * Check collision between circle and rectangle (AABB)
 */
export function circleRectCollision(
    circlePos: Vector2, radius: number,
    rectCenter: Vector2, rectWidth: number, rectHeight: number
): boolean {
    // Find closest point on rectangle to circle center
    const halfW = rectWidth / 2
    const halfH = rectHeight / 2

    const closestX = Math.max(rectCenter.x - halfW, Math.min(circlePos.x, rectCenter.x + halfW))
    const closestY = Math.max(rectCenter.y - halfH, Math.min(circlePos.y, rectCenter.y + halfH))

    const dx = circlePos.x - closestX
    const dy = circlePos.y - closestY

    return dx * dx + dy * dy < radius * radius
}

/**
 * Check collision between two robots
 */
export function checkRobotRobotCollision(
    robot1: Robot,
    robot2: Robot
): boolean {
    if (robot1.id === robot2.id) return false

    const pos1: Vector2 = { x: robot1.pose.x, y: robot1.pose.y }
    const pos2: Vector2 = { x: robot2.pose.x, y: robot2.pose.y }

    const r1 = robot1.shape.radius || 0.2
    const r2 = robot2.shape.radius || 0.2

    return circleCircleCollision(pos1, r1, pos2, r2)
}

/**
 * Check if robot collides with world boundary
 */
export function checkBoundaryCollision(
    robot: Robot,
    world: WorldConfig
): boolean {
    const radius = robot.shape.radius || 0.2
    const [offsetX, offsetY] = world.offset

    return (
        robot.pose.x - radius < offsetX ||
        robot.pose.x + radius > offsetX + world.width ||
        robot.pose.y - radius < offsetY ||
        robot.pose.y + radius > offsetY + world.height
    )
}

/**
 * Check if robot collides with any obstacle
 * Improved to handle Polygon obstacles and Rectangle robots using SAT
 */
export function checkRobotObstacleCollision(
    robot: Robot,
    obstacles: Obstacle[]
): boolean {
    // 1. Get robot polygon vertices
    let robotVertices: Vector2[] = []
    const robotPos = { x: robot.pose.x, y: robot.pose.y }

    if (robot.shape.type === 'rectangle') {
        const rotation = robot.pose.theta // Standard theta
        const length = robot.shape.height || 4.6
        const width = robot.shape.width || 1.6
        robotVertices = getRotatedRectVertices(robotPos, length, width, rotation)
    } else {
        // Quick pass: Check if robot radius collides with any obstacle
        const r = robot.shape.radius || 0.2
        for (const obs of obstacles) {
            // Skip unobstructed obstacles (walls for LiDAR only)
            if (obs.unobstructed) continue

            if (obs.shape.type === 'circle') {
                if (distance(robotPos, obs.position) < r + (obs.shape.radius || 0.5)) return true
            } else {
                // Convert obs to polygon
                let obsVertices: Vector2[] = []
                if (obs.shape.type === 'rectangle') {
                    obsVertices = getRotatedRectVertices(obs.position, obs.shape.width || 1, obs.shape.height || 1, obs.rotation || 0)
                } else if (obs.shape.type === 'polygon' && obs.shape.vertices) {
                    obsVertices = obs.shape.vertices
                }

                if (obsVertices.length > 0) {
                    if (circlePolygonCollision(robotPos, r, obsVertices)) return true
                }
            }
        }
        return false
    }

    // 2. Check collision with obstacles (SAT)
    for (const obs of obstacles) {
        // Skip unobstructed obstacles (walls for LiDAR only)
        if (obs.unobstructed) continue

        let obsVertices: Vector2[] = []
        let isCircle = false
        let circleRadius = 0
        let circlePos = { x: 0, y: 0 }

        if (obs.shape.type === 'circle') {
            isCircle = true
            circleRadius = obs.shape.radius || 0.5
            circlePos = obs.position
        } else if (obs.shape.type === 'rectangle') {
            obsVertices = getRotatedRectVertices(obs.position, obs.shape.width || 1, obs.shape.height || 1, obs.rotation || 0)
        } else if (obs.shape.type === 'polygon' && obs.shape.vertices) {
            obsVertices = obs.shape.vertices
        }

        if (isCircle) {
            if (circlePolygonCollision(circlePos, circleRadius, robotVertices)) return true
        } else {
            if (polygonPolygonCollision(robotVertices, obsVertices)) return true
        }
    }

    return false
}

// ==================== Collision Helpers ====================

function getRotatedRectVertices(center: Vector2, w: number, h: number, rotation: number): Vector2[] {
    const cos = Math.cos(rotation)
    const sin = Math.sin(rotation)
    const dx = w / 2
    const dy = h / 2

    return [
        { x: center.x + (dx * cos - dy * sin), y: center.y + (dx * sin + dy * cos) },
        { x: center.x - (dx * cos - dy * sin), y: center.y - (dx * sin + dy * cos) },
        { x: center.x - (dx * cos + dy * sin), y: center.y - (dx * sin - dy * cos) },
        { x: center.x + (dx * cos + dy * sin), y: center.y + (dx * sin - dy * cos) }
    ]
}

// SAT: Polygon vs Polygon
function polygonPolygonCollision(poly1: Vector2[], poly2: Vector2[]): boolean {
    if (poly1.length < 2 || poly2.length < 2) return false
    const polygons = [poly1, poly2]

    for (const polygon of polygons) {
        for (let i = 0; i < polygon.length; i++) {
            const p1 = polygon[i]
            const p2 = polygon[(i + 1) % polygon.length]
            const normal = { x: p2.y - p1.y, y: p1.x - p2.x }

            let minA = Infinity, maxA = -Infinity
            for (const p of poly1) {
                const proj = p.x * normal.x + p.y * normal.y
                if (proj < minA) minA = proj
                if (proj > maxA) maxA = proj
            }

            let minB = Infinity, maxB = -Infinity
            for (const p of poly2) {
                const proj = p.x * normal.x + p.y * normal.y
                if (proj < minB) minB = proj
                if (proj > maxB) maxB = proj
            }

            if (maxA < minB || maxB < minA) return false
        }
    }
    return true
}

// SAT: Circle vs Polygon
export function circlePolygonCollision(circleCenter: Vector2, radius: number, polygon: Vector2[]): boolean {
    if (polygon.length < 2) return false

    // 1. Find closest vertex on polygon to circle center
    let closestVertexDistSq = Infinity
    let closestVertexIndex = -1

    for (let i = 0; i < polygon.length; i++) {
        const dSq = (polygon[i].x - circleCenter.x) ** 2 + (polygon[i].y - circleCenter.y) ** 2
        if (dSq < closestVertexDistSq) {
            closestVertexDistSq = dSq
            closestVertexIndex = i
        }
    }

    // 2. Axes to test: Polygon edge normals AND axis from center to closest vertex
    const axes: Vector2[] = []

    // Edge normals
    for (let i = 0; i < polygon.length; i++) {
        const p1 = polygon[i]
        const p2 = polygon[(i + 1) % polygon.length]
        axes.push({ x: p2.y - p1.y, y: p1.x - p2.x })
    }

    // Axis to closest vertex
    const closestV = polygon[closestVertexIndex]
    axes.push({ x: closestV.x - circleCenter.x, y: closestV.y - circleCenter.y })

    for (const axis of axes) {
        if (axis.x === 0 && axis.y === 0) continue

        // Project polygon
        let minP = Infinity, maxP = -Infinity
        for (const p of polygon) {
            const proj = p.x * axis.x + p.y * axis.y
            if (proj < minP) minP = proj
            if (proj > maxP) maxP = proj
        }

        // Project circle
        const centerProj = circleCenter.x * axis.x + circleCenter.y * axis.y
        const axisLen = Math.sqrt(axis.x * axis.x + axis.y * axis.y)
        const rProj = radius * axisLen // Scaled projection

        const minC = centerProj - rProj
        const maxC = centerProj + rProj

        if (maxP < minC || maxC < minP) return false
    }

    return true
}

/**
 * Check if a point is inside a polygon
 */
export function pointInPolygon(point: Vector2, vertices: Vector2[]): boolean {
    let inside = false
    const n = vertices.length
    for (let i = 0, j = n - 1; i < n; j = i++) {
        const xi = vertices[i].x, yi = vertices[i].y
        const xj = vertices[j].x, yj = vertices[j].y
        if (((yi > point.y) !== (yj > point.y)) &&
            (point.x < (xj - xi) * (point.y - yi) / (yj - yi) + xi)) {
            inside = !inside
        }
    }
    return inside
}

/**
 * Circle-Linestring collision
 */
export function circleLinestringCollision(
    circleCenter: Vector2,
    radius: number,
    vertices: Vector2[]
): boolean {
    for (let i = 0; i < vertices.length - 1; i++) {
        const p1 = vertices[i]
        const p2 = vertices[i + 1]

        // Check distance from circle center to segment
        const dist = distanceToLineSegment(circleCenter, p1, p2)
        if (dist < radius) return true
    }
    return false
}

/**
 * Distance from point to line segment
 */
export function distanceToLineSegment(
    point: Vector2,
    lineStart: Vector2,
    lineEnd: Vector2
): number {
    const dx = lineEnd.x - lineStart.x
    const dy = lineEnd.y - lineStart.y
    const lengthSq = dx * dx + dy * dy

    if (lengthSq === 0) {
        return distance(point, lineStart)
    }

    let t = ((point.x - lineStart.x) * dx + (point.y - lineStart.y) * dy) / lengthSq
    t = Math.max(0, Math.min(1, t))

    const closestPoint: Vector2 = {
        x: lineStart.x + t * dx,
        y: lineStart.y + t * dy
    }

    return distance(point, closestPoint)
}
