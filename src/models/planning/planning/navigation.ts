
import { Robot, Pose, Velocity, BehaviorConfig } from './types'
import { normalizeAngle, distance, goalReached } from './utils'

// ==================== Helper Functions ====================

/**
 * Convert omnidirectional velocity to differential drive command
 */
function omniToDiff(theta: number, vel: [number, number]): Velocity {
    const vx = vel[0]
    const vy = vel[1]

    // Calculate desired heading
    const speed = Math.sqrt(vx * vx + vy * vy)
    if (speed < 0.01) {
        return { vx: 0, vy: 0, omega: 0 }
    }

    const desiredTheta = Math.atan2(vy, vx)
    let diffTheta = desiredTheta - theta

    // Normalize angle to [-π, π]
    while (diffTheta > Math.PI) diffTheta -= 2 * Math.PI
    while (diffTheta < -Math.PI) diffTheta += 2 * Math.PI

    // If angle difference is small, move forward
    const angleTolerance = 0.2
    let linear: number
    let angular: number

    if (Math.abs(diffTheta) < angleTolerance) {
        linear = speed * Math.cos(diffTheta)
        angular = 0 // Small correction already applied through cos
    } else {
        // Turn first, then move
        linear = speed * 0.3 // Reduced forward speed while turning
        angular = Math.sign(diffTheta) * Math.min(2.0, Math.abs(diffTheta) * 2)
    }

    return { vx: linear, vy: 0, omega: angular }
}

// ==================== Basic Behaviors ====================

/**
 * Dash behavior - move directly toward goal
 */
export function dashBehavior(
    currentPose: Pose,
    goal: Pose,
    maxSpeed: number
): Velocity {
    const dx = goal.x - currentPose.x
    const dy = goal.y - currentPose.y
    const dist = Math.sqrt(dx * dx + dy * dy)

    if (dist < 0.1) {
        return { vx: 0, vy: 0, omega: 0 }
    }

    // Proportional speed control
    const speed = Math.min(maxSpeed, dist)

    // Calculate desired heading
    const desiredTheta = Math.atan2(dy, dx)
    const thetaError = normalizeAngle(desiredTheta - currentPose.theta)

    // For differential drive, first turn then move
    if (Math.abs(thetaError) > 0.1) {
        return {
            vx: 0,
            vy: 0,
            omega: Math.sign(thetaError) * Math.min(2.0, Math.abs(thetaError))
        }
    }

    return {
        vx: speed,
        vy: 0,
        omega: thetaError * 0.5 // Small correction
    }
}

/**
 * Patrol behavior - cycle through multiple waypoints
 */
export function patrolBehavior(
    robot: Robot,
    patrolPoints: Pose[],
    maxSpeed: number
): { velocity: Velocity, nextGoalIndex: number } {
    if (!patrolPoints || patrolPoints.length === 0) {
        return { velocity: { vx: 0, vy: 0, omega: 0 }, nextGoalIndex: 0 }
    }

    const currentIndex = robot.currentGoalIndex || 0
    const currentGoal = patrolPoints[currentIndex]

    // Check if reached current waypoint
    if (goalReached(robot.pose, currentGoal, robot.goalThreshold || 0.3)) {
        // Move to next waypoint (cycle)
        const nextIndex = (currentIndex + 1) % patrolPoints.length
        return {
            velocity: dashBehavior(robot.pose, patrolPoints[nextIndex], maxSpeed),
            nextGoalIndex: nextIndex
        }
    }

    return {
        velocity: dashBehavior(robot.pose, currentGoal, maxSpeed),
        nextGoalIndex: currentIndex
    }
}

/**
 * Follow behavior - maintain distance from a target robot
 */
export function followBehavior(
    follower: Robot,
    leader: Robot,
    followDistance: number,
    maxSpeed: number
): Velocity {
    const dx = leader.pose.x - follower.pose.x
    const dy = leader.pose.y - follower.pose.y
    const dist = Math.sqrt(dx * dx + dy * dy)

    if (dist < followDistance * 0.9) {
        // Too close, slow down or stop
        return { vx: 0, vy: 0, omega: 0 }
    }

    // Calculate target position behind the leader
    const targetDist = dist - followDistance
    const targetX = follower.pose.x + (dx / dist) * targetDist
    const targetY = follower.pose.y + (dy / dist) * targetDist

    return dashBehavior(follower.pose, { x: targetX, y: targetY, theta: 0 }, maxSpeed)
}

/**
 * Wander behavior - random exploration within bounds
 */
export function wanderBehavior(
    robot: Robot,
    rangeLow: [number, number, number],
    rangeHigh: [number, number, number],
    maxSpeed: number
): { velocity: Velocity, newGoal?: Pose } {
    // Check if we need a new random goal
    const needNewGoal = !robot.goal || goalReached(robot.pose, robot.goal, robot.goalThreshold || 0.3)

    if (needNewGoal) {
        const newGoal: Pose = {
            x: rangeLow[0] + Math.random() * (rangeHigh[0] - rangeLow[0]),
            y: rangeLow[1] + Math.random() * (rangeHigh[1] - rangeLow[1]),
            theta: rangeLow[2] + Math.random() * (rangeHigh[2] - rangeLow[2])
        }
        return {
            velocity: dashBehavior(robot.pose, newGoal, maxSpeed),
            newGoal
        }
    }

    return {
        velocity: dashBehavior(robot.pose, robot.goal!, maxSpeed)
    }
}

// ==================== RVO (Reciprocal Velocity Obstacle) ====================

interface RVOCone {
    apex: [number, number]
    leftVector: [number, number]
    rightVector: [number, number]
}

/**
 * RVO (Reciprocal Velocity Obstacle) behavior
 */
export function rvoBehavior(
    agent: Robot,
    neighbors: Robot[],
    goal: Pose,
    maxSpeed: number,
    safetyMargin: number,
    neighborThreshold: number = 5.0
): Velocity {
    const x = agent.pose.x
    const y = agent.pose.y
    const vx = agent.velocity.vx
    const vy = agent.velocity.vy
    const radius = agent.shape.radius || 0.2

    // Desired velocity toward goal
    const dx = goal.x - x
    const dy = goal.y - y
    const goalDist = Math.sqrt(dx * dx + dy * dy)

    let vxDes = 0, vyDes = 0
    if (goalDist > (agent.goalThreshold || 0.2)) {
        const speed = Math.min(maxSpeed, goalDist)
        vxDes = (dx / goalDist) * speed
        vyDes = (dy / goalDist) * speed
    } else {
        // Reached goal
        return { vx: 0, vy: 0, omega: 0 }
    }

    // RVO state: [x, y, vx, vy, radius, vxDes, vyDes]
    const rvoState = [x, y, vx, vy, radius, vxDes, vyDes]

    // Filter neighbors within threshold
    const rvoNeighbors: number[][] = []
    for (const neighbor of neighbors) {
        if (neighbor.id === agent.id) continue

        const distSq = (neighbor.pose.x - x) ** 2 + (neighbor.pose.y - y) ** 2
        if (distSq < neighborThreshold ** 2) {
            // Neighbor state: [x, y, vx, vy, radius]
            rvoNeighbors.push([
                neighbor.pose.x,
                neighbor.pose.y,
                neighbor.velocity.vx || 0,
                neighbor.velocity.vy || 0,
                (neighbor.shape.radius || 0.2) + safetyMargin
            ])
        }
    }

    if (rvoNeighbors.length === 0) {
        // No neighbors, just go to goal
        return omniToDiff(agent.pose.theta, [vxDes, vyDes])
    }

    // Shuffle neighbors to avoid order-dependent bias
    const shuffledNeighbors = [...rvoNeighbors].sort(() => Math.random() - 0.5)

    // Build RVO cones for each neighbor
    const rvoCones: RVOCone[] = []
    for (const neighbor of shuffledNeighbors) {
        const cone = buildRVOCone(rvoState, neighbor)
        if (cone) rvoCones.push(cone)
    }

    // Sample velocity candidates
    const accel = 1.0 // Acceleration limit per step
    const vxMin = Math.max(vx - accel, -maxSpeed)
    const vxMax = Math.min(vx + accel, maxSpeed)
    const vyMin = Math.max(vy - accel, -maxSpeed)
    const vyMax = Math.min(vy + accel, maxSpeed)

    const voOutside: [number, number][] = []
    const voInside: [number, number][] = []
    const step = 0.1 // Velocity sampling resolution

    for (let newVx = vxMin; newVx <= vxMax; newVx += step) {
        for (let newVy = vyMin; newVy <= vyMax; newVy += step) {
            if (isOutsideAllCones(newVx, newVy, rvoCones)) {
                voOutside.push([newVx, newVy])
            } else {
                voInside.push([newVx, newVy])
            }
        }
    }

    // Select best velocity with tie-breaking
    let selectedVel: [number, number]

    if (voOutside.length > 0) {
        // Pick velocity outside all VOs that's closest to desired
        // Shuffle first to break ties randomly
        const shuffled = voOutside.sort(() => Math.random() - 0.5)
        selectedVel = shuffled.reduce((best, v) => {
            const bestDist = (best[0] - vxDes) ** 2 + (best[1] - vyDes) ** 2
            const vDist = (v[0] - vxDes) ** 2 + (v[1] - vyDes) ** 2
            return vDist < bestDist ? v : best
        })
    } else if (voInside.length > 0) {
        // All velocities inside VOs - pick one with minimum penalty
        const shuffled = voInside.sort(() => Math.random() - 0.5)
        selectedVel = shuffled.reduce((best, v) => {
            const bestPenalty = calculatePenalty(rvoState, rvoNeighbors, best, [vxDes, vyDes], 2.0)
            const vPenalty = calculatePenalty(rvoState, rvoNeighbors, v, [vxDes, vyDes], 2.0)
            return vPenalty < bestPenalty ? v : best
        })
    } else {
        // No candidates (shouldn't happen), use desired velocity
        selectedVel = [vxDes, vyDes]
    }

    // Convert to differential drive command
    return omniToDiff(agent.pose.theta, selectedVel)
}

function buildRVOCone(state: number[], neighbor: number[]): RVOCone | null {
    const x = state[0], y = state[1]
    const vx = state[2], vy = state[3]
    const r = state[4]

    const mx = neighbor[0], my = neighbor[1]
    const mvx = neighbor[2], mvy = neighbor[3]
    const mr = neighbor[4]

    // RVO apex is average of velocities
    const apex: [number, number] = [(vx + mvx) / 2, (vy + mvy) / 2]

    // Distance to neighbor
    let disMr = Math.sqrt((my - y) ** 2 + (mx - x) ** 2)
    const angleMr = Math.atan2(my - y, mx - x)

    // Ensure minimum distance
    const combinedRadius = r + mr
    if (disMr < combinedRadius) {
        disMr = combinedRadius
    }

    // Calculate half angle of the VO cone
    let ratio = combinedRadius / disMr
    ratio = Math.max(-1, Math.min(1, ratio)) // Clamp for asin

    const halfAngle = Math.asin(ratio)
    const lineLeftOri = angleMr + halfAngle
    const lineRightOri = angleMr - halfAngle

    const leftVector: [number, number] = [Math.cos(lineLeftOri), Math.sin(lineLeftOri)]
    const rightVector: [number, number] = [Math.cos(lineRightOri), Math.sin(lineRightOri)]

    return { apex, leftVector, rightVector }
}

function isOutsideAllCones(vx: number, vy: number, cones: RVOCone[]): boolean {
    for (const cone of cones) {
        const relVx = vx - cone.apex[0]
        const relVy = vy - cone.apex[1]

        // Check if inside this cone using cross products
        if (isBetweenVectors(cone.leftVector, cone.rightVector, [relVx, relVy])) {
            return false // Inside this VO cone
        }
    }
    return true // Outside all cones
}

function isBetweenVectors(
    leftVector: [number, number],
    rightVector: [number, number],
    testVector: [number, number]
): boolean {
    const crossLeft = leftVector[0] * testVector[1] - testVector[0] * leftVector[1]
    const crossRight = rightVector[0] * testVector[1] - testVector[0] * rightVector[1]
    return crossLeft <= 0 && crossRight >= 0
}

function calculatePenalty(
    state: number[],
    neighbors: number[][],
    vel: [number, number],
    velDes: [number, number],
    factor: number
): number {
    const tcList: number[] = []

    for (const neighbor of neighbors) {
        const distance = Math.sqrt(
            (neighbor[0] - state[0]) ** 2 + (neighbor[1] - state[1]) ** 2
        )
        const combinedRadius = state[4] + neighbor[4]

        let diff = distance ** 2 - combinedRadius ** 2
        if (diff < 0) diff = 0

        const disVel = Math.sqrt(diff)

        // Relative velocity
        const velTrans = [
            2 * vel[0] - state[2] - neighbor[2],
            2 * vel[1] - state[3] - neighbor[3]
        ]
        const velTransSpeed = Math.sqrt(velTrans[0] ** 2 + velTrans[1] ** 2) + 1e-7

        const tc = disVel / velTransSpeed
        tcList.push(tc)
    }

    let tcMin = Math.min(...tcList)
    if (tcMin === 0) tcMin = 0.0001

    // Penalty = factor * (1/time_to_collision) + distance_to_desired_velocity
    const distToDes = Math.sqrt((velDes[0] - vel[0]) ** 2 + (velDes[1] - vel[1]) ** 2)
    return factor * (1 / tcMin) + distToDes
}

// ==================== ORCA (Optimal Reciprocal Collision Avoidance) ====================

interface Vector2Simple { x: number, y: number }

/**
 * ORCA (Optimal Reciprocal Collision Avoidance) behavior
 * More efficient version using half-plane constraints
 */
export function orcaBehavior(
    agent: Robot,
    neighbors: Robot[],
    goal: Pose,
    config: {
        neighborDist: number
        maxNeighbors: number
        timeHorizon: number
        timeHorizonObst: number
        safeRadius: number
        maxSpeed: number
    }
): Velocity {
    const agentPos: Vector2Simple = { x: agent.pose.x, y: agent.pose.y }
    const agentVel: Vector2Simple = { x: agent.velocity.vx, y: agent.velocity.vy }
    const agentRadius = agent.shape.radius || 0.2

    // Preferred velocity toward goal
    const dx = goal.x - agentPos.x
    const dy = goal.y - agentPos.y
    const goalDist = Math.sqrt(dx * dx + dy * dy)

    let prefVx = 0, prefVy = 0
    if (goalDist > 0.1) {
        const speed = Math.min(config.maxSpeed, goalDist)
        prefVx = (dx / goalDist) * speed
        prefVy = (dy / goalDist) * speed
    }

    // Find nearby neighbors (sorted by distance)
    const nearbyNeighbors = neighbors
        .filter(n => n.id !== agent.id)
        .map(n => ({
            robot: n,
            dist: distance({ x: agentPos.x, y: agentPos.y }, { x: n.pose.x, y: n.pose.y })
        }))
        .filter(n => n.dist < config.neighborDist)
        .sort((a, b) => a.dist - b.dist)
        .slice(0, config.maxNeighbors)

    if (nearbyNeighbors.length === 0) {
        return { vx: prefVx, vy: prefVy, omega: 0 }
    }

    // ORCA lines (half-plane constraints)
    const orcaLines: { point: Vector2Simple, direction: Vector2Simple }[] = []

    for (const { robot: neighbor } of nearbyNeighbors) {
        const neighborPos: Vector2Simple = { x: neighbor.pose.x, y: neighbor.pose.y }
        const neighborVel: Vector2Simple = { x: neighbor.velocity.vx, y: neighbor.velocity.vy }
        const neighborRadius = neighbor.shape.radius || 0.2

        const relPos = { x: neighborPos.x - agentPos.x, y: neighborPos.y - agentPos.y }
        const relVel = { x: agentVel.x - neighborVel.x, y: agentVel.y - neighborVel.y }
        const combinedRadius = agentRadius + neighborRadius + config.safeRadius

        const invTimeHorizon = 1.0 / config.timeHorizon
        const relPosScaled = { x: relPos.x * invTimeHorizon, y: relPos.y * invTimeHorizon }
        const combinedRadiusScaled = combinedRadius * invTimeHorizon

        // Calculate u (closest point on VO boundary to relative velocity)
        const w = { x: relVel.x - relPosScaled.x, y: relVel.y - relPosScaled.y }
        const wLengthSq = w.x * w.x + w.y * w.y

        if (wLengthSq > combinedRadiusScaled * combinedRadiusScaled) {
            // Not in collision cone
            const wLength = Math.sqrt(wLengthSq)
            const unitW = { x: w.x / wLength, y: w.y / wLength }
            const uLength = combinedRadiusScaled - wLength
            const u = { x: unitW.x * uLength, y: unitW.y * uLength }

            orcaLines.push({
                point: { x: agentVel.x + u.x * 0.5, y: agentVel.y + u.y * 0.5 },
                direction: { x: -unitW.y, y: unitW.x }
            })
        }
    }

    // Linear programming to find safe velocity closest to preferred
    let newVelX = prefVx
    let newVelY = prefVy

    for (const line of orcaLines) {
        // Project preferred velocity onto half-plane if needed
        const dot = (prefVx - line.point.x) * (-line.direction.y) + (prefVy - line.point.y) * line.direction.x
        if (dot < 0) {
            // Outside half-plane, project onto line
            const t = (prefVx - line.point.x) * line.direction.x + (prefVy - line.point.y) * line.direction.y
            newVelX = line.point.x + t * line.direction.x
            newVelY = line.point.y + t * line.direction.y
        }
    }

    // Limit speed
    const speed = Math.sqrt(newVelX * newVelX + newVelY * newVelY)
    if (speed > config.maxSpeed) {
        newVelX = (newVelX / speed) * config.maxSpeed
        newVelY = (newVelY / speed) * config.maxSpeed
    }

    return { vx: newVelX, vy: newVelY, omega: 0 }
}
