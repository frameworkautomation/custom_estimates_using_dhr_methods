import React, { useMemo } from 'react'
import { Canvas } from '@react-three/fiber'
import { OrbitControls } from '@react-three/drei'
import Waypoint from './Waypoint'
import Edges from './Edges'

// Small axis-cross marker for origin points
function OriginMarker({ position, color, size = 80 }) {
  const [x, y, z] = position
  const half = size / 2
  return (
    <group>
      <mesh position={[x, y, z]}>
        <sphereGeometry args={[size * 0.4, 12, 12]} />
        <meshStandardMaterial color={color} emissive={color} emissiveIntensity={0.4} />
      </mesh>
      {/* X axis */}
      <mesh position={[x + half, y, z]} rotation={[0, 0, Math.PI / 2]}>
        <cylinderGeometry args={[size * 0.08, size * 0.08, size, 6]} />
        <meshStandardMaterial color="#ff4444" />
      </mesh>
      {/* Y axis (scene up = robot Z) */}
      <mesh position={[x, y + half, z]}>
        <cylinderGeometry args={[size * 0.08, size * 0.08, size, 6]} />
        <meshStandardMaterial color="#44ff44" />
      </mesh>
      {/* Z axis (scene depth = robot Y) */}
      <mesh position={[x, y, z + half]} rotation={[Math.PI / 2, 0, 0]}>
        <cylinderGeometry args={[size * 0.08, size * 0.08, size, 6]} />
        <meshStandardMaterial color="#4488ff" />
      </mesh>
    </group>
  )
}

export default function Scene({ waypoints, edges, selected, onSelect, origins }) {
  const waypointMap = useMemo(() => {
    const m = {}
    waypoints.forEach((wp) => { m[wp.name] = wp })
    return m
  }, [waypoints])

  // Convert robot coords → scene coords: [robot_x, robot_z, robot_y]
  const worldPos = [0, 0, 0]
  const robotBasePos = origins?.robot_base
    ? [origins.robot_base.x, origins.robot_base.z, origins.robot_base.y]
    : null

  return (
    <Canvas
      camera={{ position: [4000, 2500, 3000], fov: 55, near: 1, far: 200000 }}
      style={{ width: '100%', height: '100%', background: '#1a1a2e' }}
      onPointerMissed={() => {}}
    >
      <ambientLight intensity={0.7} />
      <directionalLight position={[5000, 8000, 5000]} intensity={0.9} />
      <OrbitControls target={[1500, 800, 0]} makeDefault />

      {/* World origin — white */}
      <OriginMarker position={worldPos} color="#ffffff" size={100} />

      {/* Robot base origin — orange (shown only when robot_base_world is in path_config.yaml) */}
      {robotBasePos && (
        <OriginMarker position={robotBasePos} color="#ff8800" size={100} />
      )}

      {waypoints.map((wp) => (
        <Waypoint
          key={wp.name}
          waypoint={wp}
          isSelected={selected.includes(wp.name)}
          onSelect={onSelect}
        />
      ))}

      <Edges edges={edges} waypointMap={waypointMap} />
    </Canvas>
  )
}
