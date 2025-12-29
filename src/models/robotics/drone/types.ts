
export interface Vector2 {
    x: number
    y: number
}

export interface Pose {
    x: number
    y: number
    theta: number // orientation in radians
}

export interface Velocity {
    vx: number
    vy: number
    omega: number // angular velocity
}

export type KinematicsType = 'differential' | 'omni' | 'ackermann' | 'quadrotor'
export type ShapeType = 'circle' | 'rectangle' | 'polygon' | 'linestring'
export type BehaviorType = 'dash' | 'rvo' | 'orca' | 'patrol' | 'follow' | 'wander' | 'custom'
export type DistributionType = 'manual' | 'random' | 'circle' | 'grid'
export type CollisionMode = 'stop' | 'unobstructed' | 'unobstructed_obstacles'

export interface RobotShape {
    type: ShapeType
    radius?: number // for circle
    width?: number  // for rectangle
    height?: number // for rectangle (length in ir-sim)
    vertices?: Vector2[] // for polygon and linestring
    wheelbase?: number // for ackermann
}

// FOV (Field of View) sensor configuration
export interface FOVConfig {
    angle: number // field of view angle in radians
    range: number // maximum detection range
    enabled: boolean
}

// Kinematics noise model (α model for odometry error)
export interface KinematicsNoise {
    enabled: boolean
    alpha: [number, number, number, number] // [α1, α2, α3, α4]
}

// Behavior configuration
export interface BehaviorConfig {
    type: BehaviorType
    vxmax?: number
    vymax?: number
    acce?: number
    factor?: number
    wander?: boolean
    wanderRange?: { low: [number, number, number], high: [number, number, number] }
    patrolPoints?: Pose[] // for patrol behavior
    followTarget?: string // robot id to follow
    followDistance?: number
    neighborThreshold?: number // for RVO/ORCA
    timeHorizon?: number
    timeHorizonObst?: number
    safeRadius?: number
}

// Distribution configuration for multi-robot spawning
export interface DistributionConfig {
    type: DistributionType
    center?: Vector2
    radius?: number
    rangeLow?: [number, number, number]
    rangeHigh?: [number, number, number]
    nonOverlapping?: boolean
    positions?: Pose[] // for manual distribution
}

export interface Robot {
    id: string
    pose: Pose
    velocity: Velocity
    shape: RobotShape
    kinematics: KinematicsType
    color: string
    goal?: Pose
    goals?: Pose[] // support multiple waypoints
    currentGoalIndex?: number
    trajectory: Vector2[]
    maxSpeed: number
    wheelBase?: number
    // Extended properties
    behavior?: BehaviorConfig
    fov?: FOVConfig
    noise?: KinematicsNoise
    estimatedPose?: Pose // pose with noise (for visualization)
    detectedRobots?: string[] // IDs of robots in FOV
    arriveMode?: 'position' | 'angle'
    goalThreshold?: number
    velMin?: [number, number]
    velMax?: [number, number]
    acce?: [number, number]
}

export interface Obstacle {
    id: string
    position: Vector2
    shape: RobotShape
    color: string
    velocity?: Vector2 // for dynamic obstacles
    goal?: Pose // obstacles can have goals too
    behavior?: BehaviorConfig
    kinematics?: KinematicsType
    unobstructed?: boolean // pass-through obstacle
    rotation?: number // orientation for rectangles/polygons
}

export interface WorldConfig {
    width: number
    height: number
    offset: [number, number]
    stepTime: number
    sampleTime?: number
    collisionMode?: CollisionMode
    controlMode?: 'auto' | 'keyboard'
    obstacleMap?: string // path to obstacle map image
}

export interface LiDARConfig {
    maxRange: number
    minRange?: number
    fov: number // degrees
    resolution: number // degrees per ray
    noise?: number // standard deviation
    angleNoise?: number
    offset?: [number, number, number] // x, y, theta offset from robot center
    alpha?: number // rendering transparency
}

export interface LiDARScan {
    angles: number[] // radians
    distances: number[]
    detected: boolean[]
    hitPoints?: Vector2[]
}

export interface SimulationState {
    world: WorldConfig
    robots: Robot[]
    obstacles: Obstacle[]
    time: number
    step: number
    isPaused?: boolean
    keyboardState?: KeyboardState
}

// Keyboard control state
export interface KeyboardState {
    up: boolean
    down: boolean
    left: boolean
    right: boolean
    linearSpeed: number
    angularSpeed: number
    targetRobotId?: string
}

// Plot/Rendering configuration
export interface PlotConfig {
    showTrajectory?: boolean
    showTrail?: boolean
    trailFill?: boolean
    trailAlpha?: number
    keepTrailLength?: number
    showGoal?: boolean
    showText?: boolean
    showFov?: boolean
    showArrow?: boolean
    arrowLength?: number
    goalColor?: string
    trajColor?: string
}

// Map related types
export interface GridMap {
    data: number[][] // 0: free, 1: occupied
    resolution: number
    origin: Vector2
    width: number
    height: number
}
