import React, { useMemo } from 'react'
import { Canvas } from '@react-three/fiber'
import { OrbitControls, Html } from '@react-three/drei'
import Waypoint from './Waypoint'
import Edges from './Edges'

// XYZ axis-cross marker for world/robot origins
function OriginMarker({ position, label, size = 150 }) {
  const [x, y, z] = position
  const h = size / 2
  return (
    <group>
      {/* Centre sphere */}
      <mesh position={[x, y, z]}>
        <sphereGeometry args={[size * 0.35, 14, 14]} />
        <meshStandardMaterial color="#ffffff" emissive="#ffffff" emissiveIntensity={0.6} />
      </mesh>

      {/* X axis — red */}
      <mesh position={[x + h, y, z]} rotation={[0, 0, Math.PI / 2]}>
        <cylinderGeometry args={[size * 0.07, size * 0.07, size, 6]} />
        <meshStandardMaterial color="#ff3333" />
      </mesh>
      {/* Y axis (scene up = robot Z) — green */}
      <mesh position={[x, y + h, z]}>
        <cylinderGeometry args={[size * 0.07, size * 0.07, size, 6]} />
        <meshStandardMaterial color="#33cc33" />
      </mesh>
      {/* Z axis (scene depth = robot Y) — blue */}
      <mesh position={[x, y, z + h]} rotation={[Math.PI / 2, 0, 0]}>
        <cylinderGeometry args={[size * 0.07, size * 0.07, size, 6]} />
        <meshStandardMaterial color="#3388ff" />
      </mesh>

      {/* Floating text label */}
      <Html position={[x, y + size * 0.9, z]} center style={{ pointerEvents: 'none' }}>
        <span style={{
          color: '#ffffff', background: 'rgba(0,0,0,0.6)',
          padding: '2px 6px', borderRadius: 4,
          fontSize: 12, fontFamily: 'monospace', whiteSpace: 'nowrap',
        }}>
          {label}
        </span>
      </Html>
    </group>
  )
}

// Colour legend overlay (outside Canvas, positioned absolutely)
const LEGEND_ITEMS = [
  { color: '#44bb44', label: 'Waypoint' },
  { color: '#ffff00', label: 'Waypoint — selected' },
  { color: '#ffffff', label: 'World origin' },
  { color: '#ff8800', label: 'Robot base' },
  { color: '#44bb44', label: 'Edge — collision free' },
  { color: '#ff4444', label: 'Edge — collision detected' },
  { color: '#555577', label: 'Edge — untested' },
]

function Legend() {
  return (
    <div style={{
      position: 'absolute', bottom: 16, right: 16,
      background: 'rgba(0,0,0,0.72)', borderRadius: 8,
      padding: '10px 14px', fontSize: 12, fontFamily: 'monospace',
      color: '#fff', pointerEvents: 'none', zIndex: 10,
      lineHeight: '1.7',
    }}>
      <div style={{ marginBottom: 4, fontWeight: 'bold', fontSize: 13 }}>Legend</div>
      {LEGEND_ITEMS.map(({ color, label }) => (
        <div key={label} style={{ display: 'flex', alignItems: 'center', gap: 8 }}>
          <div style={{
            width: 12, height: 12, borderRadius: '50%',
            background: color, flexShrink: 0,
          }} />
          {label}
        </div>
      ))}
      <div style={{ marginTop: 6, borderTop: '1px solid #444', paddingTop: 6, fontSize: 11, color: '#aaa' }}>
        Axes on each point: X=red Y=green Z=blue
      </div>
    </div>
  )
}

export default function Scene({ waypoints, edges, selected, onSelect, origins }) {
  const waypointMap = useMemo(() => {
    const m = {}
    waypoints.forEach((wp) => { m[wp.name] = wp })
    return m
  }, [waypoints])

  // Scene mapping: [-robot_x, robot_z, robot_y]
  const toScene = ({ x = 0, y = 0, z = 0 }) => [-x, z, y]
  const worldPos    = toScene(origins?.world      ?? {})
  const robotBasePos = origins?.robot_base ? toScene(origins.robot_base) : null

  return (
    <div style={{ position: 'relative', width: '100%', height: '100%' }}>
      <Canvas
        camera={{ position: [-4000, 2500, 3000], fov: 55, near: 1, far: 200000 }}
        style={{ width: '100%', height: '100%', background: '#1a1a2e' }}
        onPointerMissed={() => {}}
      >
        <ambientLight intensity={0.7} />
        <directionalLight position={[5000, 8000, 5000]} intensity={0.9} />
        <OrbitControls target={[-1500, 800, 0]} makeDefault />

        {/* World origin marker — always shown */}
        <OriginMarker position={worldPos} label="World origin" size={150} />

        {/* Robot base marker — shown only once robot_base_world is in path_config.yaml */}
        {robotBasePos && (
          <OriginMarker position={robotBasePos} label="Robot base" size={150} />
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

      <Legend />
    </div>
  )
}
