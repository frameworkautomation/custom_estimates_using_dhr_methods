import React, { useMemo } from 'react'
import { Canvas } from '@react-three/fiber'
import { OrbitControls } from '@react-three/drei'
import Waypoint from './Waypoint'
import Edges from './Edges'

export default function Scene({ waypoints, edges, selected, onSelect }) {
  const waypointMap = useMemo(() => {
    const m = {}
    waypoints.forEach((wp) => { m[wp.name] = wp })
    return m
  }, [waypoints])

  return (
    <Canvas
      camera={{ position: [4000, 2500, 3000], fov: 55, near: 1, far: 200000 }}
      style={{ width: '100%', height: '100%', background: '#1a1a2e' }}
      onPointerMissed={() => {}}
    >
      <ambientLight intensity={0.7} />
      <directionalLight position={[5000, 8000, 5000]} intensity={0.9} />
      <OrbitControls target={[1500, 800, 0]} makeDefault />

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
